/* 
 * MicroStorm - BLE Tracking
 * src/particle.c
 *
 * Copyright (c) 2022 Ricardo Steijn
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
#include <math.h>
#include <string.h>
#include <float.h>

#include "particle.h"
#include "util.h"
#include "config.h"

/**
 * \brief Normalize probability weights of particles.
 * 
 * \param arr Array of particles.
 * \param size Size of the array.
 */
static void 
ble_particle_normalize(ble_particle_t *arr, int size)
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
static ble_particle_t *
ble_particle_generate(int size)
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
                (particles+(p-1))->state.pos.x = scaled_x;
                break;
            case 1:
                scaled_y = ble_util_scale(sample[p], 0, 1, 0, AREA_Y);
                (particles+(p-1))->state.pos.y = scaled_y;
                break;
            default:
                break;
            }
            // sample angle in range [0..2*pi]
            (particles+(p-1))->state.theta = ble_util_sample_range(0.0F, (2.0F * M_PI));
            // inital motion state
            (particles+(p-1))->state.motion = MOTION_STATE_STOP;
            // initial (normalized) weight value
            (particles+(p-1))->weight = 1.0F / PARTICLE_SET;
        }
        free(sample);
    }
    free(primes);

    return particles;
}

/**
 * \brief Create a sample from Gaussian probability distribution,
 * using the Box-Muller algorithm.
 * 
 * \param mu Mean of the Gaussian.
 * \param sigma Standarddeviation.
 * 
 * \return Value in Gaussian distribution with given mu and sigma.
 */
static float 
ble_particle_gaussian_sample(float mu, float sigma)
{
    // sample 2 numbers in the range [0..1]
    // make sure u1 is greater than machine epsilon
    float u1, u2;
    do
    {
        u1 = ble_util_sample_range(0.0F, 1.0F);
        u2 = ble_util_sample_range(0.0F, 1.0F);
    }
    while (u1 <= FLT_EPSILON);

    // calculate the x value
    float mag = sigma * sqrtf(-2.0F * logf(u1));
    float z0 = mag * cosf((2.0F * M_PI) * u2) + mu;
    
    return z0;
}

/**
 * \brief Predict a new state for each particle according to 
 * motion, orientation and position models.
 * 
 * \param particles Array of particles.
 * \param size Size of the particle set.
 */
static void 
ble_particle_state_predict(ble_particle_t *particles, int size)
{
    for (int i = 0; i < size; i++) {
        float d_theta = 0, d_pos = 0;
        // sample a motion state for every particle
        ble_particle_motion_t m_sample = 
            (ble_particle_motion_t)ble_util_sample(MOTION_STATE_COUNT);
        // sample orientation delta en position delta based on motion state
        switch(m_sample) {
        case MOTION_STATE_STOP:
            // orientation sampled in range [0..2*pi], postion unchanged
            d_theta = ble_util_sample_range(0.0F, (2.0F * M_PI));
            break;
        case MOTION_STATE_MOVING:
            // orientation and position sampled from Gaussian distribution
            d_theta = ble_particle_gaussian_sample(0.0F, sqrtf(ORIENTATION_VAR));
            d_pos = fabsf(ble_particle_gaussian_sample(POSITION_MEAN, sqrtf(POSITION_VAR)));
            break;
        default:
            break;
        }
        // calculate new position and project back in area when out of bounds
        particles[i].state.pos.x = clampf(particles[i].state.pos.x + 
            (d_pos * cosf(particles[i].state.theta)), 0, AREA_X);
        particles[i].state.pos.y = clampf(particles[i].state.pos.y + 
            (d_pos * sinf(particles[i].state.theta)), 0, AREA_Y);
        // set new motion state and calculate new orientation within unit circle
        particles[i].state.motion = m_sample;
        particles[i].state.theta = clampaf(particles[i].state.theta + d_theta);
    }
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
static float 
ble_particle_weight_gain(ble_particle_ap_dist_t *dist, int size)
{
    // longest estimated distance amongst states
    ble_particle_ap_dist_t max_dist = dist[0];
    for (int i = 1; i < size; i++) {
        if (dist[i].d_node > max_dist.d_node)
            max_dist = dist[i];
    }
    // calculate average variance between a particle and each AP
    float d_diff = 0, norm_d_est = 0, norm_d = 0;
    for (int i = 0; i < size; i++) {
        // normalize distances to better represent the differences
        // x_norm = (x - x_min) / (x_max - x_min), where x_min is always 0
        norm_d_est = dist[i].d_node / max_dist.d_node;
        norm_d = dist[i].d_particle / sqrtf(powf(AREA_X, 2) + powf(AREA_Y, 2));
        // summation of absolute normalizated distance differences
        d_diff += fabsf(norm_d - norm_d_est);
    }
    d_diff /= size;
    // calculate gain factor based on Gaussian distribution
    // g(x)_t = exp(-1/2 * (D_t / m_noise_ap)^2)
    return expf(-0.5F * powf((d_diff / AP_MEASUREMENT_VAR), 2));
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
static void 
ble_particle_resample(ble_particle_t *particles, int size)
{
    ble_particle_t *new_particles = calloc(size, sizeof(ble_particle_t));
    if (new_particles == NULL)
        return;

    int pos = 0;
    // sample a value in range [0..1/N]
    float start = ble_util_sample_range(0.0F, (1.0F / (float)size));
    // generate an array of pointers using this value (according to SUS spec)
    int index = 0;
    float sum = particles[index].weight;
    for (int k = 0; k < size; k++) {
        float pointer = start + ((float)k * (1.0F / (float)size));
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
 * \brief Update the weights of each particle
 * once a new set of RSSI measurements is received.
 * Following Monte Carlo's localization model.
 * 
 * \param data Pointer to a structure with AP measurements
 * and the current postion state of the node
 * 
 * \return 0 on succes, -1 on failure.
 */
int 
ble_particle_update(ble_particle_data_t *data)
{
    static ble_particle_t *particles = NULL;
    static ble_particle_ap_t *prev_ap = NULL;

    // generate a new set of particles, uniformly distributed over area
    // only when not yet initialized
    if (particles == NULL) {
        // weights are initalized based on the starting position of the node
        particles = ble_particle_generate(PARTICLE_SET);
        // allocation error
        if (particles == NULL)
            return -1;
    }

    // predict new state for all particles according to motion models
    ble_particle_state_predict(particles, PARTICLE_SET);

    // calculate exact distance from AP to each particle
    // and gain factor according to observation model
    ble_particle_ap_dist_t **dist; 
    dist = malloc(PARTICLE_SET * sizeof(ble_particle_ap_dist_t*));
    if (dist == NULL)
        return -1;
    for (int i = 0; i < PARTICLE_SET; i++) {
        dist[i] = malloc(NO_OF_APS * sizeof(ble_particle_ap_dist_t));
        if (dist[i] == NULL)
            return -1;
    }
    for (int i = 0; i < PARTICLE_SET; i++) {
        for (int j = 0; j < NO_OF_APS; j++) {
            // use absolute distance to access point, direction not important here
            float d_diff_x = fabsf(data->aps[j].pos.x - particles[i].state.pos.x);
            float d_diff_y = fabsf(data->aps[j].pos.y - particles[i].state.pos.y);
            // assuming our area is rectangualar
            // using Pythagorean theorem: a^2 + b^2 = c^2
            dist[i][j].d_particle = sqrtf(powf(d_diff_x, 2) + powf(d_diff_y, 2));
            dist[i][j].d_node = data->aps[j].node_distance;
        } 
    }
    for (int i = 0; i < PARTICLE_SET; i++) {
        float gain = ble_particle_weight_gain(dist[i], NO_OF_APS);
        // calculate new weight for each particle
        particles[i].weight = particles[i].weight * gain;
    }
    // normalize weights again so that the sum equals 1
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
        sum_coord_x += (particles[i].weight * particles[i].state.pos.x);
        sum_coord_y += (particles[i].weight * particles[i].state.pos.y);
    }
    // clamp position in our area
    data->node.pos.x = clampf((sum_coord_x / sum_weights), 0, AREA_X);
    data->node.pos.y = clampf((sum_coord_y / sum_weights), 0, AREA_Y);

    if (prev_ap == NULL)
        prev_ap = malloc(NO_OF_APS * sizeof(ble_particle_ap_t));
    // overwrite previous state    
    memcpy(prev_ap, data->aps, NO_OF_APS * sizeof(ble_particle_ap_t));
    
    for (int i = 0; i < PARTICLE_SET; i++)
        free(dist[i]);
    free(dist);

    return 0;
}