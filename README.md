# Notes
1. It's a lightweight 3d kdtree library based on nanoflann: https://github.com/jlblancoc/nanoflann.
2. Both StaticKdTree3d and DynamicKdTree3d supports for nearestKSearch and radiusSearch.
3. StaticKdTree3d is similar to pcl::KdTreeFLANN&lt;pcl::PointXYZ&gt; but faster.
4. DynamicKdTree3d supports for dynamic updating(i.e. 3d pointcloud map building) without rebuilding the whole tree.
5. Since nanoflann is a C++11 header-only library, you can just copy the hpp files and test/use it.


# Building 
1.  install the Eigen3
2.  cd ~/catkin_ws/src  
    git clone https://github.com/G3tupup/KdTree.git  
    cd ..  
    catkin_make -DCMAKE_BUILD_TYPE=release
    or  
    you can compile dynamic_kd_tree_3d_example.cpp and static_kd_tree_3d_example.cpp in anyway you like.

# Run
1. cd ~/catkin_ws/devel/lib/kdtree
2. ./dynamic_kd_tree_3d_example and ./dynamic_kd_tree_3d_example