/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.h 3293 2005-11-19 08:37:45Z gerkey $
 *************************************************************************/

#ifndef PF_H
#define PF_H

#include "pf_vector.h"
#include "pf_kdtree.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct _pf_t;
struct _rtk_fig_t;
struct _pf_sample_set_t;

// Function prototype for the initialization model; generates a sample pose from
// an appropriate distribution.
// 函数指针的重命名,
// pf_init_model_fn_t是函数指针，(*)后面是的括号是函数的形参，
// void *表示无类型指针，可以指向任意类型的变量; (*)前面pf_vector_t值得是这个函数的返回类型。
typedef pf_vector_t (*pf_init_model_fn_t) (void *init_data);
//       返回类型            函数名                参数表

// Function prototype for the action model; generates a sample pose from
// an appropriate distribution
typedef void (*pf_action_model_fn_t) (void *action_data, 
                                      struct _pf_sample_set_t* set);

// Function prototype for the sensor model; determines the probability
// for the given set of sample poses.
typedef double (*pf_sensor_model_fn_t) (void *sensor_data, 
                                        struct _pf_sample_set_t* set);


// Information for a single sample
// 单个采样粒子：位姿+权重信息
typedef struct
{
  // Pose represented by this sample
  pf_vector_t pose;

  // Weight for this pose
  double weight;
  
} pf_sample_t;


// Information for a cluster of samples
//聚类后的粒子集的信息：数目+粒子的总权重+均值和方差
typedef struct
{
  // Number of samples
  int count;

  // Total weight of samples in this cluster
  double weight;

  // Cluster statistics
  pf_vector_t mean;
  pf_matrix_t cov;

  // Workspace
  double m[4], c[2][2];
  
} pf_cluster_t;


// Information for a set of samples
// 采样的粒子集：粒子数+单个粒子+kdtree+均值和方差，集合的大小
typedef struct _pf_sample_set_t
{
  // The samples
  int sample_count;
  pf_sample_t *samples;

  // A kdtree encoding the histogram
  pf_kdtree_t *kdtree;

  // Clusters
  int cluster_count, cluster_max_count;//粒子数，最大粒子数，达到最大粒子数，是kld里面自适应的实现么？
  pf_cluster_t *clusters; //落在同一bin的集合

  // Filter statistics
  pf_vector_t mean;
  pf_matrix_t cov;
  int converged; 
  double n_effective;
} pf_sample_set_t;


// Information for an entire filter
typedef struct _pf_t
{
  // This min and max number of samples
  int min_samples, max_samples;

  // Population size parameters
  double pop_err, pop_z;

  // Resample limit cache
  int *limit_cache;
  
  // The sample sets.  We keep two sets and use [current_set]
  // to identify the active set.
  int current_set;
  pf_sample_set_t sets[2];

  // Running averages, slow and fast, of likelihood
  // 表示粒子集平均权重的变化程度
  double w_slow, w_fast;

  // Decay rates for running averages
  double alpha_slow, alpha_fast;

  // Function used to draw random pose samples
  pf_init_model_fn_t random_pose_fn;
  void *random_pose_data;

  double dist_threshold; //distance threshold in each axis over which the pf is considered to not be converged
  int converged; 

  // boolean parameter to enamble/diable selective resampling
  int selective_resampling;
} pf_t;

//pf_alloc和pf_init的区别是：
//1、前者用drand48函数，在地图的空白区域产生随机均匀分布的粒子，后者是用pf_pdf_gaussian_sample函数，
//   在给定位姿Pose的附近产生的随机正态分布的粒子;
//2、前者的cluster的均值和协方差是0，后者pf_cluster_stats函数根据pdf的值更新了cluster的均值和协方差;
//3、前者是程序必须要执行的操作，后者只在特定场景用，比如给定了初始值，才调用。给定初值便于滤波器快速收敛。

// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn, void *random_pose_data);

// Free an existing filter
void pf_free(pf_t *pf);

// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov);

// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data);

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data);

// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data);

// Resample the distribution
void pf_update_resample(pf_t *pf);
void new_pf_update_resample(pf_t *pf, pf_sample_set_t *camera_cloud);


// set selective resampling parameter
void pf_set_selective_resampling(pf_t *pf, int selective_resampling);

// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var);

// Compute the statistics for a particular cluster.  Returns 0 if
// there is no such cluster.
int pf_get_cluster_stats(pf_t *pf, int cluster, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov);

// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t *pf, pf_sample_set_t *set);


// Display the sample set
void pf_draw_samples(pf_t *pf, struct _rtk_fig_t *fig, int max_samples);

// Draw the histogram (kdtree)
void pf_draw_hist(pf_t *pf, struct _rtk_fig_t *fig);

// Draw the CEP statistics
void pf_draw_cep_stats(pf_t *pf, struct _rtk_fig_t *fig);

// Draw the cluster statistics
void pf_draw_cluster_stats(pf_t *pf, struct _rtk_fig_t *fig);

//calculate if the particle filter has converged - 
//and sets the converged flag in the current set and the pf 
int pf_update_converged(pf_t *pf);

//sets the current set and pf converged values to zero
void pf_init_converged(pf_t *pf);

void pf_copy_set(pf_sample_set_t* set_a, pf_sample_set_t* set_b);

#ifdef __cplusplus
}
#endif


#endif
