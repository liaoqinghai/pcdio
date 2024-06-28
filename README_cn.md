# pcdio
## 什么是 `pcdio`？
`pcdio`是一个仅包含头文件的库`header-only`，用于读取和写入 PCD 文件，除了标准库没有任何其他依赖。
它的代码来自[PCL 1.14.1](https://github.com/PointCloudLibrary/pcl/commit/b0b25194f214112e3eb3db876f33960fd85f5794)的IO模块，并进行了一些修改，目的是去除了对 Boost、Eigen 和其他 PCL 模块的依赖。

pcdio 的功能与 PCL IO模块相同（只是整合PCL代码）：
- 支持最高到 V0.7 版本的 PCD 格式
- 支持 ascii、binary、binary_compressed
- 使用相同的 pcl 命名空间和 API
- 使用 pcl::PCLPointCloud2 表示点云

pcdio 在 `Ubuntu 20.04 x86_64` 上进行了测试，应该也可以在其他平台上运行(没有测试)。

## 为什么创建这个库？
PCL PCD 扩展性很好，在激光SLAM、自动驾驶都已经广泛使用，已经是一种通用的点云格式。
但PCL是一个庞大的库，依赖太多并且不能单独安装模块。

## 如何实现的？
- 将代码合并到一个单一的头文件中
- 用 std 异常替换 pcl 异常
- 添加自定义的 `Vector4f` 和 `Quaternionf` 以去除对 `Eigen` 的依赖
- 添加一些代码以去除对 `Boost` 的依赖
  - 实现 `iequals`、`trim`、`IsBigEndian`
  - 放弃文件锁定功能（可能会导致问题）

## 如何使用？
`demo.cpp` 是一个演示如何使用它的示例。它的作用是：读取 .pcd 文件，打印元信息，然后将读入的点云写入一个新文件`new.pcd`（ascii，默认精度）。

```bash
$ gcc demo.cpp -I ./ -o demo -lstdc++
# 生成可执行文件：demo
$ ./demo ./data/xyzit_binary_compressed.pcd 
# 打印元信息
# 生成 new.pcd 
$ md5sum new.pcd ./data/xyzit_ascii_precision_8.pcd 
c104bfd8c0abfb5619a833d823dc5db2  new.pcd
c104bfd8c0abfb5619a833d823dc5db2  ./data/xyzit_ascii_precision_8.pcd
```
只需关注三样：
- `pcl::PCLPointCloud2`：用于存储点云数据的容器
- `pcl::io::loadPCDFile`：加载 pcd 文件
- `pcl::io::savePCDFile`：保存 pcd 文件

另外如果要允许打印log的话，需要在`#include "pcdio/pcdio.h"`之前定义宏：`#define PCDIO_ENABLE_CONSOLE_LOG`


## 待办事项
- [ ] 提供一个简便方法将`pcl::PCLPointCloud2`转换为自定义数据结构，例如在pcl中使用 `pcl::PointCloud<PointT>`
