/* 
 * MicroStorm - BLE Tracking
 * src/particle.c
 *
 * Copyright (c) 2021 Ricardo Steijn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "particle.h"
#include "util.h"

/**
 * \brief Plot the particle and node positions in SerialPlot.
 * 
 * \param particles Array of particles.
 * \param node Node state.
 */
void ble_particle_plot(ble_particle_t *particles, ble_mqtt_node_state_t node)
{
    for (int i = 0; i < PARTICLE_SET; i++) {
        printf("%c,%g,%g\n", 'p', particles[i].state.coord.x, particles[i].state.coord.y);   
    }
    printf("%c,%g,%g\n", 'n', node.coord.x, node.coord.y);
}

/**
 * \brief Normalize probability weights of particles.
 * 
 * \param arr Array of particles.
 * \param size Size of the array.
 */
void ble_particle_normalize(ble_particle_t *arr, int size)
{
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i].weight;
    }
    // normalize such that particles are within 0..1
    // and sum of all particles equals 1
    // though it may not be exactly 1, because of floating point inaccuracy
    for (int i = 0; i < size; i++) {
        arr[i].weight = arr[i].weight / sum;
    }
}

/**
 * \brief Uniformly Generate particles across the known area 
 * using Halton sequence. https://en.wikipedia.org/wiki/Halton_sequence
 * 
 * \param size Amount of particles to be generated.
 * 
 * \return Pointer to an array of uniformly generated particles.
 * Returns NULL on error.
 */
ble_particle_t *ble_particle_generate(int size)
{
    ble_particle_t *particles = calloc(size, sizeof(ble_particle_t));
    if (particles == NULL)
        return NULL;

    int set_size = size + 1, dim = 2;
    float scaled_x, scaled_y;
    // get N prime numbers using Sieve of Eratosthenes
    // we only need 2 here, as our dimensions are 2D
    int *primes = ble_util_prime_sieve(dim);
    if (primes == NULL)
        return NULL;
    // generate van der corput samples
    for (int i = 0; i < dim; i++) {
        float *sample = ble_util_corput(set_size, primes[i]);
        if (sample == NULL) {
            free(primes);
            return NULL;
        }
        // save x and y coordinates respectively
        // scale values from (0,0 -> 1,1) to our area
        for (int p = 1; p < set_size; p++) {
            switch(i) {
            case 0:
                scaled_x = ble_util_scale(sample[p], 0, 1, 0, AREA_X);
                (particles+(p-1))->state.coord.x = scaled_x;
                break;
            case 1:
                scaled_y = ble_util_scale(sample[p], 0, 1, 0, AREA_Y);
                (particles+(p-1))->state.coord.y = scaled_y;
                break;
            }
            // random angle in a full circle, in radians
            (particles+(p-1))->state.angle = ble_util_rand_float(0, (2 * M_PI));
            // initial weight value
            (particles+(p-1))->weight = (float)1 / PARTICLE_SET;
        }
        free(sample);
    }
    free(primes);

    return particles;
}

/**
 * \brief Stochastic Universal Sampling (SUS) algorithm
 * to resample all particles, where particles with a higher weight
 * have a higher chance of being reproduced, so we only keep the best particles.
 * This mitigates inaccuracy overtime.
 * 
 * \param particles Array of particles.
 * \param size Size of particle set.
 */
void ble_particle_resample(ble_particle_t *particles, int size)
{
    ble_particle_t *new_particles = calloc(size, sizeof(ble_particle_t));
    if (new_particles == NULL)
        return;

    int pos = 0;
    // choose a random value
    float start = ble_util_rand_float(0, (1 / (float)size));
    // generate an array of pointers using this 1 random variable (according to SUS spec)
    int index = 0;
    float sum = particles[index].weight;
    for (int k = 0; k < size; k++) {
        float pointer = start + ((float)k * (1 / (float)size));
        // reproduce particles with higher weights
        // higher weight means the sum is higher than the pointer for a few iterations
        // and the same particle is included multiple times
        while (sum < pointer) {
            index++;
            sum += particles[index].weight;
        }
        new_particles[pos++] = particles[index];
    }
    // normalize weights so that the sum is equal to 1 again
    ble_particle_normalize(new_particles, size);
    // overwrite old particles
    memcpy(particles, new_particles, size * sizeof(ble_particle_t));
    free(new_particles);
}

/**
 * \brief Update the weights of a particle
 * once a new set of RSSI measurements is received.
 * 
 * \param dist Pointer to an array of estimates for each access point.
 * \param size Array size of dist.
 * 
 * \return Weight gain factor for a particle.
 */
float ble_particle_weight_gain(ble_particle_ap_dist_t *dist, int size)
{
    // calculate mean distance difference between a particle and each AP
    float d_diff = 0;
    for (int i = 0; i < size; i++) {
        // summation of distance differences to the power of 2
        d_diff += powf(fabsf(dist[i].d_particle - dist[i].d_node), 2);
    }
    d_diff /= size;
    // calculate gain factor based on Gaussian distribution
    // g(x)_t = exp(-1/2 * (D_t / m_noise_ap)^2)
    return expf(-0.5 * d_diff / powf(AP_MEASUREMENT_NOISE, 2));
}

/**
 * \brief Update the weights of each particle
 * once a new set of RSSI measurements is received.
 * Following Monte Carlo's localization model.
 * 
 * \param ap Pointer to an array with ap specific data.
 * \param node Pointer to struct with current state of the node.
 * \param size Size of the array of RSSI states.
 * 
 * \return 0 on succes, -1 on failure.
 */
int ble_particle_update(ble_mqtt_pf_data_t *data)
{
    static ble_particle_t *particles = NULL;
    static ble_mqtt_ap_t *prev_ap = NULL;
    static int64_t start_us = 0; 

    // generate a new set of particles, uniformly distributed over area
    // only when not yet initialized
    if (particles == NULL) {
        // weights are initalized based on the starting position of the node
        particles = ble_particle_generate(PARTICLE_SET);
        // allocation error
        if (particles == NULL)
            return -1;
    }

    // allocate memory for distance info
    ble_particle_ap_dist_t **dist; 
    dist = malloc(PARTICLE_SET * sizeof(ble_particle_ap_dist_t*));
    if (dist == NULL)
        return -1;
    for (int i = 0; i < PARTICLE_SET; i++) {
        dist[i] = malloc(NO_OF_APS * sizeof(ble_particle_ap_dist_t));
        if (dist[i] == NULL)
            return -1;
    }

    // calculate particle distances to AP's
    for (int i = 0; i < PARTICLE_SET; i++) {
        for (int j = 0; j < NO_OF_APS; j++) {
            // use absolute distance to access point, direction not important here
            float d_diff_x = fabsf(data->aps[j].pos.x - particles[i].state.coord.x);
            float d_diff_y = fabsf(data->aps[j].pos.y - particles[i].state.coord.y);
            // assuming our area is rectangualar
            // using Pythagorean theorem: a^2 + b^2 = c^2
            dist[i][j].d_particle = sqrtf(powf(d_diff_x, 2) + powf(d_diff_y, 2));
            dist[i][j].d_node = data->aps[j].node_distance;
        } 
    }

    // skip if theres are no previous states to compare
    if (prev_ap != NULL) {
        float heading_vec_x = 0, heading_vec_y = 0;
        float dt = (float)US_TO_S(ble_util_timedelta(&start_us));
        // guard division by zero or incorrect time value
        if (dt <= 0)
            return -1;
        // calculate direction and magnitude of the heading vector
        // by adding all the vectors pointing to each AP
        for (int i = 0; i < NO_OF_APS; i++) {
            float angle, gradient;
            // calculate angle from x axis counterclockwise to each AP
            angle = ble_util_angle_2_pi((data->aps[i].pos.y - data->node.coord.y), 
                (data->aps[i].pos.x - data->node.coord.x));
            // calculate gradient between previous state and new state
            gradient = (data->aps[i].node_distance - prev_ap[i].node_distance) / dt;
            // calculate the sum of the x and y vectors
            // reverse direction of the vectors, negetive gradient means heading towards AP
            heading_vec_x += (-1 * (gradient * cosf(angle)));
            heading_vec_y += (-1 * (gradient * sinf(angle)));
        }
        // update node speed (magnitude of heading vector) and angle estimates
        float speed = sqrtf(powf(heading_vec_x, 2) + powf(heading_vec_y, 2));
        float angle = ble_util_angle_2_pi(heading_vec_y, heading_vec_x);
        
        // update the state of each particle
        for (int i = 0; i < PARTICLE_SET; i++) {
            // the rotation change is proportional to that of the node
            float new_angle = particles[i].state.angle + (angle - data->node.angle);
            // make sure to be in the unit circle
            new_angle = (new_angle < 0) ? (new_angle + (2 * M_PI)) : new_angle;
            new_angle = (new_angle > (2 * M_PI)) ? (new_angle - (2 * M_PI)): new_angle;
            // calculate translation in x direction using angle
            float shift_x = (speed * dt) * cosf(new_angle);
            float new_x = particles[i].state.coord.x + shift_x + GAUSS_NOISE_X;
            // calculate translation in y direction using angle
            float shift_y = (speed * dt) * sinf(new_angle);
            float new_y = particles[i].state.coord.y + shift_y + GAUSS_NOISE_Y;
            // set new values
            particles[i].state.angle = new_angle;
            particles[i].state.coord.x = clampf(new_x, 0, AREA_X);
            particles[i].state.coord.y = clampf(new_y, 0, AREA_Y);
        }
        // set new node values
        data->node.speed = speed;
        data->node.angle = angle;
    }

    // weight gain per particle (caculated by distance differences)
    for (int i = 0; i < PARTICLE_SET; i++) {
        float gain = ble_particle_weight_gain(dist[i], NO_OF_APS);
        // calculate new weight for each particle
        float new_weight = particles[i].weight * gain;
        particles[i].weight = new_weight;
    }
    ble_particle_normalize(particles, PARTICLE_SET);

    // calculate effective sample size (ESS) for normalized weights where
    // w_i >= 0 and sum(w_i) -> N with i = 1 equals 1
    // ESS = 1 / sum(w_i)^2 -> N
    float sum_weights_pow = 0;
    for (int i = 0; i < PARTICLE_SET; i++)
        sum_weights_pow += powf(particles[i].weight, 2);
    float n_eff = 1 / sum_weights_pow;
    // check if we need to resample based on effective sample size
    if (n_eff < (PARTICLE_SET * RATIO_COEFFICIENT))
        ble_particle_resample(particles, PARTICLE_SET);

    // calculate a weighted average of all particles for a node state estimate
    float sum_coord_x = 0, sum_coord_y = 0, sum_weights = 0;
    for (int i = 0; i < PARTICLE_SET; i++) {
        sum_weights += particles[i].weight;
        sum_coord_x += (particles[i].weight * particles[i].state.coord.x);
        sum_coord_y += (particles[i].weight * particles[i].state.coord.y);
    }
    // clamp position in our area
    data->node.coord.x = clampf((sum_coord_x / sum_weights), 0, AREA_X);
    data->node.coord.y = clampf((sum_coord_y / sum_weights), 0, AREA_Y);

    if (prev_ap == NULL)
        prev_ap = malloc(NO_OF_APS * sizeof(ble_mqtt_ap_t));
    // overwrite previous state    
    memcpy(prev_ap, data->aps, NO_OF_APS * sizeof(ble_mqtt_ap_t));
    
    for (int i = 0; i < PARTICLE_SET; i++)
        free(dist[i]);
    free(dist);

    ble_particle_plot(particles, data->node);

    return 0;
}