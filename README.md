# *Weighted Poisson-disk Resampling on Large-Scale Point Clouds*

Created by X. Jiao and C. Lv. The paper is accepted by AAAI.

![image](https://github.com/user-attachments/assets/421cf13e-778d-487c-aae7-9a8b6e09edca)

## Introduction

For large-scale point cloud processing, resampling takes the important role of controlling the point number and density while keeping the geometric consistency. % in related tasks. 
However, current methods cannot balance such different requirements. Particularly with large-scale point clouds, classical methods often struggle with decreased efficiency and accuracy. To address such issues, we propose a weighted Poisson-disk (WPD) resampling method to improve the usability and efficiency for the processing. We first design an initial Poisson resampling with a voxel-based estimation strategy. It is able to estimate a more accurate radius of the Poisson-disk while maintaining high efficiency. Then, we design a weighted tangent smoothing step to further optimize the Voronoi diagram for each point. At the same time, sharp features are detected and kept in the optimized results with isotropic property. Finally, we achieve a resampling copy from the original point cloud with the specified point number, uniform density, and high-quality geometric consistency. Experiments show that our method significantly improves the performance of large-scale point cloud resampling for different applications, and provides a highly practical solution.

## Citation
If you find our work useful in your research, please consider citing:

     @article{Jiao2025wpd,
         title={Weighted Poisson-disk Resampling on Large-Scale Point Clouds},
         author={Xianhe Jiao, Chenlei Lv, Junli Zhao, Ran Yi, Yu-Hui Wen, Zhenkuan Pan, Zhongke Wu, Yong-jin Liu}
         journal={arXiv preprint arXiv:XXXXXXXX},
         year={2025}
     }

## Implementation
This project is implemented by VS2022, C++. We release the whole project. You can change the set based on our instance.

Addtional libiaries should be included:

PCL library 1.13.1: https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.13.1-rc1

VCG library: https://github.com/cnr-isti-vclab/vcglib

OpenCV 3.4.6: https://github.com/opencv/opencv/releases/tag/3.4.6

## Update

2024/12/11 The project of WPD is created. 

2024/12/10 The paper of WPD is accepted by AAAI. 
