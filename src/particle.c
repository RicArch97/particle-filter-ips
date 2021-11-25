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
#include "rssi.h"
#include "util.h"

/**
 * \brief Uniformly Generate particles across the known area 
 * using Halton sequence. https://en.wikipedia.org/wiki/Halton_sequence
 * 
 * \return Pointer to an array of uniformly generated particles.
 * Returns NULL on error.
 */
ble_particle_t *ble_particle_generate(float start_x, float start_y)
{
    ble_particle_t *particles = calloc(PARTICLE_SET_SIZE, sizeof(ble_particle_t));
    if (particles == NULL)
        return NULL;
    
    // sample points using Halton sequence
    int size = PARTICLE_SET_SIZE + 1, dim = 2;
    float scaled_x, scaled_y;
    // get N prime numbers using Sieve of Eratosthenes
    // we only need 2 here, as our dimensions are 2D
    int *primes = ble_util_prime_sieve(dim);
    if (primes == NULL)
        return NULL;
    // generate van der corput samples
    for (int i = 0; i < dim; i++) {
        float *sample = ble_util_corput(size, primes[i]);
        if (sample == NULL) {
            free(primes);
            return NULL;
        }
        // save x and y coordiantes respectively
        // scale values from (0,0 -> 1,1) to our area
        for (int p = 1; p < size; p++) {
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
            // TODO
            // generated inital weights for all particles
        }
        free(sample);
    }
    free(primes);

    return particles;
}

/**
 * \brief Predict a new state for all particles.
 * 
 * \param p Array with all particles.
 * \param size Array size of particles.
 */
void ble_particle_state_predict(ble_particle_t *p, float speed, int size)
{
    static int64_t start_us = 0;
    // calculate timedelta using high resolution timer
    float dt = ble_util_timedelta(&start_us);

    for (int i = 0; i < size; i++) {
        // calculate translation in x direction using angle alpha (hypotenuse)
        float shift_x = (speed * dt) * cosf(p[i].state.angle);
        float new_x = p[i].state.coord.x + shift_x + GAUSS_NOISE_X;
        // calculate translation in y direction using angle alpha (hypotenuse)
        float shift_y = (speed * dt) * sinf(p[i].state.angle);
        float new_y = p[i].state.coord.y + shift_y + GAUSS_NOISE_Y;
        // set new position
        p->state.coord.x = new_x;
        p->state.coord.y = new_y;
    }
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
 * \brief Stochastic Universal Sampling (SUS) algorithm
 * to resample all particles, where particles with a higher weight
 * have a higher chance of being reproduced, so we only keep the best particles.
 * This mitigates inaccuracy overtime.
 * 
 * \param particles Array of particles.
 * \param size Size of particle set.
 * 
 * \return Pointer to array of resampled particles.
 * Returns NULL on error.
 */
ble_particle_t *ble_particle_resample(ble_particle_t *particles, int size)
{
    ble_particle_t *new_particles = calloc(size, sizeof(ble_particle_t));
    if (new_particles == NULL)
        return NULL;

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
    ble_particle_normalize(new_particles, size);
    return new_particles;
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
    float d_diff = 0;
    // calculate mean distance difference between a particle and each AP
    for (int i = 0; i < size; i++) {
        float norm_d = dist[i].d_particle / sqrtf(powf(AREA_X, 2) + powf(AREA_Y, 2));
        // longest estimated distance amongst states
        ble_particle_ap_dist_t max_dist = dist[0];
        for (int j = 1; j < size; j++) {
            if (dist[i].d_node > max_dist.d_node)
                max_dist = dist[i];
        }
        float norm_d_est = dist[i].d_node / max_dist.d_node;
        // summation of normalizated distance differences
        d_diff += (norm_d - norm_d_est);
    }
    d_diff /= size;

    // calculate gain factor based on Gaussian distribution
    // g(x)_t = exp(-1/2 * (D_t / m_noise_ap)^2)
    return expf(-0.5 * powf((d_diff / AP_MEASUREMENT_NOISE), 2));
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
 * \return target state estimation on succes, -1 on failure.
 */
int ble_particle_update(ble_mesh_ap_t *ap, ble_mesh_node_state_t *node, int size)
{
    static ble_particle_t *particles = NULL;
    static ble_mesh_ap_t *prev_ap = NULL;
    static int64_t start_us = 0;

    // generate a new set of particles, uniformly distributed over area
    // only when not yet initialized
    if (particles == NULL) {
        // weights are initalized based on the starting position of the node
        particles = ble_particle_generate(node->coord.x, node->coord.y);
        // allocation error
        if (particles == NULL)
            return -1;
    }

    // allocate memory for distance info
    int dist_size = (size * PARTICLE_SET_SIZE) * sizeof(ble_particle_ap_dist_t);
    ble_particle_ap_dist_t *dist = malloc(dist_size);
    if (dist == NULL)
        return -1;

    // calculate particle distances to AP's
    for (int i = 0; i < PARTICLE_SET_SIZE; i++) {
        for (int j = 0; j < size; j++) {
            // use absolute distance to access point, direction not important here
            float d_diff_x = abs(ap[j].pos.x - particles[i].state.coord.x);
            float d_diff_y = abs(ap[j].pos.y - particles[i].state.coord.y);
            // assuming our area is rectangualar
            // using Pythagorean theorem: a^2 + b^2 = c^2
            (dist+i+j)->d_particle = sqrtf(powf(d_diff_x, 2) + powf(d_diff_y, 2));
            (dist+i+j)->d_node = ap[j].node_distance;
        } 
    }

    // skip if theres are no previous states to compare
    if (prev_ap != NULL) {
        float dt = ble_util_timedelta(&start_us);
        // calculate direction and magnitude of the heading vector
        // by adding all the vectors pointing to each AP
        float sum_vec_x = 0, sum_vec_y = 0;
        for (int i = 0; i < size; i++) {
            float angle, dy, gradient;
            // calculate angle from x axis counterclockwise to each AP
            angle = ble_util_angle_2_pi((ap[i].pos.y - node->coord.y), 
                        (ap[i].pos.x - node->coord.x));
            // calculate gradient between previous state and new state
            dy = ap[i].node_distance - prev_ap[i].node_distance;
            gradient = dy / dt;
            // calculate the sum of the x and y vectors
            sum_vec_x += (gradient * cosf(angle));
            sum_vec_y += (gradient * sinf(angle));
        }
        // update node speed (magnitude of heading vector) and angle estimates
        // heading angle is not used in PF, but could be used to simulate the node
        node->speed = sqrtf(powf(sum_vec_x, 2) + powf(sum_vec_y, 2));
        node->angle = ble_util_angle_2_pi(sum_vec_y, sum_vec_x);
    }

    // particles move with the same speed as the node
    ble_particle_state_predict(particles, node->speed, PARTICLE_SET_SIZE);
    int index = 0;
    // weight gain per particle (caculated by distance differences)
    for (int i = 0; i < (size * PARTICLE_SET_SIZE); i += size) {
        float gain = ble_particle_weight_gain((dist+i), size);
        // calculate new weight for each particle
        float new_weight = particles[index].weight * gain;
        particles[index++].weight = new_weight;
    }
    ble_particle_normalize(particles, PARTICLE_SET_SIZE);

    // calculate effective sample size (ESS) for normalized weights where
    // w_i >= 0 and sum(w_i) -> N with i = 1 equals 1
    // ESS = 1 / sum(w_i)^2 -> N
    float sum_weights_pow = 0;
    for (int i = 0; i < PARTICLE_SET_SIZE; i++) {
        sum_weights_pow += powf(particles[i].weight, 2);
    }
    float n_eff = 1 / sum_weights_pow;
    // check if we need to resample based on effective sample size
    if (n_eff < (PARTICLE_SET_SIZE * RATIO_COEFFICIENT)) {
        ble_particle_t *resampled = ble_particle_resample(particles, PARTICLE_SET_SIZE);
        if (resampled == NULL)
            return -1;
        // free previous allocated block for particles and point to new block
        free(particles);
        particles = resampled;
    }

    // average the coordinates of existing particles from their coordinate vectors
    float sum_coord_x = 0, sum_coord_y = 0;
    for (int i = 0; i < PARTICLE_SET_SIZE; i++) {
        sum_coord_x += particles[i].state.coord.x;
        sum_coord_y += particles[i].state.coord.y;
    }
    node->coord.x = (sum_coord_x / PARTICLE_SET_SIZE);
    node->coord.y = (sum_coord_y / PARTICLE_SET_SIZE);

    // update previous state
    if (prev_ap != NULL) {
        free(prev_ap);
        prev_ap = ap;
    }
    free(dist);

    return 0;
}