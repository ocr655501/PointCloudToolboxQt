##Point Cloud Toolbox 
点云工具箱（点云处理程序）

based on Qt 5.12.10, Point Cloud Library 1.12 with an additional Vtk 8.2.
##这是什么？
在业余时间完成的，算是图像处理工具的一个程序样品。对部分点云数据能进行一些初步处理的操作。

- 有用户界面，操作较为简便。
- 只用 CPU，部分工控机可用。
- 可以个性化显示窗口部分参数。



- 教育意义较大，参数较为详细，参数可配置。
- 可以处理的格式较多。

**最低运行分辨率：1366 * 768**

**建议运行分辨率：1920 * 1080**


##Issues (BUG)
- Surface 设备会出现 DPI 错误

其他设备应该影响不大。这个问题可以使用系统兼容性选项修复。
- 在某些特殊机型或者配置怪异的计算机可能会运行不正常。

比如，部分强行安装 Windows 11 的计算机可能部分功能无法正常使用。
- 部分操作程序可能闪退。

调试后告知为「堆已损坏」。

- 中文路径无法读取

可以解决。但是由于个人时间紧迫，没有采取解决方案...
##先决条件
以 Windows 系统为例：
- 下载并安装最新版本的 Point Cloud Library

`https://pointclouds.org/downloads/`

> 不建议通过除百度外的中国大陆搜索引擎下载。要么搜不到，要么绕个大弯，要么直接是另外一个简称为 PCL 的 Pure Minecraft Launcher 

> Point Cloud Library 也简称 PCL。但是最好不要直接搜索 PCL，这可不是 Pure Minecraft Launcher 这个同样简称为 PCL 的 MC 启动器

- 从  [Releases](https://github.com/ocr655501/PointCloudToolboxQt/releases)  中下载独立的 vtk8.2 依赖库。

> 由于一些原因，中国大陆用户可能无法正常下载 Releases 中的内容，请提前备好网络环境，或者直接划到文章最底下

- 打开高级系统设置，在环境变量「PATH」里面添加这些内容：

```
%PCLPATH%\3rdParty\FLANN\bin
%PCLPATH%\3rdParty\Qhull\bin
%PCLPATH%\3rdParty\VTK\bin

再看看 %PCLPATH%\3rdParty\OpenNI2 里面是有几个文件夹，还是只有一个安装包

如果里面有几个文件夹，就添加这些
%PCLPATH%\3rdParty\OpenNI2\Tools
%PCLPATH%\3rdParty\OpenNI2\Redist

如果孤零零的只有一个安装包的话，添加这些
C:\Program Files\OpenNI2\Tools
C:\Program Files\OpenNI2\Redist

%PCLPATH% 指的是 Point Cloud Library 的安装目录。
假设我的电脑安装了 PCL 1.12.1，安装目录为 F:\Program Files\PCL 1.12.1
就把 %PCLPATH% 这一串内容更换为 F:\Program Files\PCL 1.12.1
```

在完成这些操作之后，接下来的步骤分为两个不同分支：

####我想直接使用程序

- 从  [Releases](https://github.com/ocr655501/PointCloudToolboxQt/releases)  中下载软件压缩包。
- 解压到自己喜欢的位置，然后把之前下载的 VTK 库一起解压进去。这就 OK 了 

####我想自己编译、研究和学习

- 使用 Git 克隆项目到计算机

`git clone https://github.com/ocr655501/PointCloudToolboxQt`

- 计算机中需要安装 Visual Studio 2019 或更新版本，Qt 5.10 或更新版本

Visual Studio 2019 是主IDE，Qt 5.10 是用来界面设计的。

- 从  [Releases](https://github.com/ocr655501/PointCloudToolboxQt/releases)  中下载独立的 vtk8.2 依赖库，解压到一个**能够方便被添加到变量**的位置，最好是不要带中文。


- 打开解决方案文件，**修改**属性中的以下内容：

VC++ 目录：包含目录

```
%PCLPATH%\3rdParty\FLANN\include
%PCLPATH%\OpenNI2\Include
// C:\Program Files\OpenNI2\Include 如果上面那个目录不存在，就用这个
%PCLPATH%\3rdParty\Qhull\include
%PCLPATH%\3rdParty\Boost\include\boost-1_78
%PCLPATH%\include\pcl-1.12
%PCLPATH%\3rdParty\Eigen\eigen3
%vtk_path%\include\vtk-8.2
%PCLPATH%\3rdParty\VTK\include\vtk-9.1

%PCLPATH% 指的是 Point Cloud Library 的安装目录。
假设我的电脑安装了 PCL 1.12.1，安装目录为 F:\Program Files\PCL 1.12.1
就把 %PCLPATH% 这一串内容更换为 F:\Program Files\PCL 1.12.1

%vtk_path% 指的是刚才你解压 vtk 的目录。比如：
我解压 vtk 到了 D:\VTK，就把 %vtk_path% 改成 D:\VTK，比如
D:\VTK\include\vtk-8.2
```

VC++ 目录：库目录 ~~（LNK2019）~~

```
%PCLPATH%\OpenNI2\Lib
// C:\Program Files\OpenNI2\Lib 如果上面那个目录不存在，就用这个
%PCLPATH%\3rdParty\FLANN\lib
%PCLPATH%\3rdParty\Boost\lib
%PCLPATH%\lib
%PCLPATH%\3rdParty\Qhull\lib
%vtk_path%\lib
%PCLPATH%\3rdParty\VTK\lib

%PCLPATH% 指的是 Point Cloud Library 的安装目录。
假设我的电脑安装了 PCL 1.12.1，安装目录为 F:\Program Files\PCL 1.12.1
就把 %PCLPATH% 这一串内容更换为 F:\Program Files\PCL 1.12.1

%vtk_path% 指的是刚才你解压 vtk 的目录。比如：
我解压 vtk 到了 D:\VTK，就把 %vtk_path% 改成 D:\VTK，比如
D:\VTK\include\vtk-8.2
```

- 理论来说这样应该没问题了。如果还有问题，就再按照下面的操作处理：

C/C++：预处理器：预处理器定义

- 复制这串内容覆盖
```
_SCL_SECURE_NO_WARNINGS
BOOST_USE_WINDOWS_H
NOMINMAX
_CRT_SECURE_NO_DEPRECATE
_SILENCE_FPOS_SEEKPOS_DEPRECATION_WARNING
```

链接器：输入：附加依赖项

`把项目里面 AdditionalDependicies.txt 的内容无脑扔进去就行`

- 最后还要在 Visual Studio 里面安装并配置 Qt VS Tools 扩展用于联合编译。

> 既然你都已经来到这里了，需要什么必要条件估计我就不用说了吧。留意你的网络环境。

- 最后看看这个工程是否完整。


- 生成。这就 OK 了

##声明

本程序有借助部分论坛的学习笔记编写，算是一组学习笔记的整合版本。欢迎研究代码学习点云处理和深度计算。

由于个人时间紧张，没法对程序进行进一步改进，因此我做出了一个决定，允许 Fork 项目方便后继者进行进一步的改进。**前提是不要涉及著作权问题，不要商业化本程序。这种行为不但是对软件作者、学习笔记原作者的不尊重，更是对开源的极度不尊重。**

**不要让我看到有人卖这款软件，这是对所有学习笔记作者的极大不尊重。**

##国内下载地址
众所周知，githubcontents 近几年在国内屡遭黑名单屏蔽，如果你下载不了 Releases 的程序实例，可以试试这里。

[下载整个项目 / 压缩包](https://wwb.lanzoub.com/icEY405n2bsf)

[下载 Windows 可执行文件 / 压缩包](https://wwb.lanzoub.com/iTZQB05n2lzc)

[下载 vtk 库 / 压缩包](https://wwb.lanzoub.com/iZnkj05n38ch)
