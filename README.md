# Notes
1. It's a lightweight head-only 3d kdtree library based on nanoflann: https://github.com/jlblancoc/nanoflann.
2. Both StaticKdTree3d and DynamicKdTree3d supports for nearestKSearch and radiusSearch.
3. StaticKdTree3d is similar to pcl::KdTreeFLANN&lt;pcl::PointXYZ&gt; but faster.
4. DynamicKdTree3d supports for dynamic updating(i.e. 3d pointcloud map building) without rebuilding the whole tree.
5. Both StaticKdTree3d and DynamicKdTree3d supports for float/double/etc point type.


# Building 
1.  install the Eigen3
2.  install pcl if you want to build and run time_compare.cpp
3.  cd ~/catkin_ws/src  
    git clone https://github.com/G3tupup/KdTree.git  
    cd ..  
    catkin_make -DCMAKE_BUILD_TYPE=release
    or  
    you can compile dynamic_3d_tree_example.cpp/static_3d_tree_example.cpp/time_compare in anyway you like.

# Run
1. cd ~/catkin_ws/devel/lib/kdtree
2. ./dynamic_3d_tree_example and ./dynamic_3d_tree_example 
3. ./time_compare to compare StaticKdTree3d with pcl::KdTreeFLANN ![my result](../result/time_compare.jpg)