# DynaHull

DynaHull: Density-centric Dynamic Point Filtering in Point Clouds

In the field of indoor robotics, accurately localizing and mapping in dynamic environments using point clouds can be a challenging task due to the presence of dynamic points. These dynamic points are often represented by people in indoor environments, but in industrial settings with moving machinery, there can be various types of dynamic points. This study introduces DynaHull, a novel technique designed to enhance indoor mapping accuracy by effectively removing dynamic points from point clouds. DynaHull works by leveraging the observation that, over multiple scans, stationary points have a higher density compared to dynamic ones. Furthermore, DynaHull addresses mapping challenges related to unevenly distributed points by clustering the map into smaller sections. In each section, the density factor of each point is determined by dividing the number of neighbors by the volume these neighboring points occupy using a convex hull method. The algorithm removes the dynamic points using an adaptive threshold based on the point count of each cluster, thus reducing the false positives. The performance of DynaHull was compared to state-of-the-art techniques, such as ERASOR, Removert, OctoMap+ SOR (statistical outlier removal), and Dynablox, by comparing each method to the ground truth map created during a low activity period in which only a few dynamic points were present. The results indicated that DynaHull outperformed these techniques in various metrics, noticeably in the Earth Mover’s Distance, false positives, and negatives. This research contributes to indoor robotics by providing efficient methods for dynamic point removal, essential for accurate mapping and localization in dynamic environments.

![Estimating convex hull volume between stationary and dynamic points](img/fdsc.png)
*Estimating convex hull volume between stationary and dynamic points: The left figure represents the stationary area (wall), while the picture on the right illustrates the dynamic area (human)*

## Required Libraries
- numpy
- scikit-learn
- open3d
- scipy
- glob
- os
- configparser

## Citation
@misc{habibiroudkenar2024dynahull,
title={DynaHull: Density-centric Dynamic Point Filtering in Point Clouds},
author={Pejman Habibiroudkenar and Risto Ojala and Kari Tammi},
year={2024},
eprint={2401.07541},
archivePrefix={arXiv},
primaryClass={cs.RO}
}
