# pcdio

[中文](./README_cn.md)

## What is pcdio?
`pcdio` is a **header-only** library for reading and writing PCD files **without any dependencies** except for the standard library. 

It's code comes from [PCL 1.14.1](https://github.com/PointCloudLibrary/pcl/commit/b0b25194f214112e3eb3db876f33960fd85f5794) IO module with some modifications which removes the dependency on Boost, Eigen and other PCL modules.

`pcdio` acts same as PCL IO module (just re-organize PCL code): 
- supports the PCD formats version up to V0.7 
- supports `ascii`, `binary`, `binary_compressed`
- use same pcl namespace and api
- use `pcl::PCLPointCloud2` to represent the point cloud


`pcdio` has been tested on `Ubuntu 20.04 x86_64`, it should work on other platforms.

## Why create this?
First of all, PCL PCD format is good engouth and widely used (e.g. LiDAR SLAM and Autonomous driving). 
But PCL is a heavy library which has losts of denpendencies and we cannot install single module.  


## How I make it?
- merge code into one sinle header file
- replace pcl exception with std exception
- add custom `Vector4f` and `Quaternionf` to remove `Eigen` dependency
- add some code to remove `Boost` dependency
  - implement `iequals` , `trim`, `IsBigEndian`
  - give up the file lock feature (may cause problem)

## How to use it?
`demo.cpp` is a demo to show how to use it. The demo does: 
1. read `.pcd` file
2. print the meta info 
3. save loaded pointcloud to `new.pcd` (ascii, default precision)
4. convert the loaded pointcloud to custom pointcloud data structure(KITTI BIN format) and save to KITTI bin file

```bash
$ gcc demo.cpp -I ./ -o demo -lstdc++
# get exe file: demo
$ ./demo ./data/xyzit_binary_compressed.pcd 
# print meta info
# get new.pcd 
# get kitti.bin
$ md5sum new.pcd ./data/xyzit_ascii_precision_8.pcd 
c104bfd8c0abfb5619a833d823dc5db2  new.pcd
c104bfd8c0abfb5619a833d823dc5db2  ./data/xyzit_ascii_precision_8.pcd
```


Just care about three things:
- `pcl::PCLPointCloud2` : a container for storing point cloud data
- `pcl::io::loadPCDFile()` : load pcd file
- `pcl::io::savePCDFile()` : save pcd file

if you want to allow it print log, please define `PCDIO_ENABLE_CONSOLE_LOG` macro before include `pcdio/pcdio.h`


### How to use my custom pointcloud data structure?
`pcl::PCLPointCloud2`  is the structure to present the pointcloud from PCD file. Unfortunately, we do not provide an easy way like PCL lib to convert between `pcl::PCLPointCloud2` and `pcl::PointCloud<PointT>` which support to register custom point type.
We have to implement the convert function by ourselves. In the `demo.cpp` we show the example to use KITTI point type, you can refer the two function:
- `bool FromPCLPointCloud2(const pcl::PCLPointCloud2& pc2, KittiPointCloud& pointcloud)` : read pcd file and use this to convert  `pcl::PCLPointCloud2` to your own data
- `bool ToPCLPointCloud2(const KittiPointCloud& pointcloud, pcl::PCLPointCloud2& pc2)` : use this to convert you own pointcloud data to `pcl::PCLPointCloud2` and then save as pcd file


## TODO
- [ ] provide easy way to convert `pcl::PCLPointCloud2` to custom data structure, like use `pcl::PointCloud<PointT>`
