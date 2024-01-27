# Shenzhen_innoX_2024wc_ws

## 更新说明

镜像仓库地址：https://hub.docker.com/repository/docker/wzx1210/innox2024_wc

| 时间 | tag | 说明 |
| --------- | ------ | ------------------------------------------------------------------------- |
| 2024.1.17 | wzx1210/innox2024_wc:v1.0.1 | 更新镜像支持可视化展示、支持"/dev/robomaster"（未测试）、支持"/dev/video"（未测试）|
| 2024.1.18 | wzx1210/innox2024_wc:v1.0.2 | 安装缺少库，支持摄像头 |
| 2024.1.24 | wzx1210/innox2024_wc:v1.0.3 | 安装putty等串口调试工具 |
| 2024.1.24 | wzx1210/innox2024_wc:v1.0.4 | 安装串口库 |

| 2024.1.24 | wzx1210/innox2024_wc:v2.0.0 | 【弃用】基础镜像：arm64v8/ubuntu:focal |
| 2024.1.24 | wzx1210/innox2024_wc:v2.0.1 | 【弃用】更换系统源，安装ros，安装python并换源，修改系统时区，安装git、vim，putty等常用工具 |
| 2024.1.24 | wzx1210/innox2024_wc:v2.0.2 | 【弃用】配置工作空间环境 |
| 2024.1.24 | wzx1210/innox2024_wc:v2.0.3 | 【弃用】支持可视化 |

| 时间 | tag | 说明 |
| --------- | ------ | ------------------------------------------------------------------------- |
| 2024.1.26 | registry.cn-hangzhou.aliyuncs.com/wzx1210/innox2024_wc:v3.0.0 |  ros基础镜像|
| 2024.1.26 | registry.cn-hangzhou.aliyuncs.com/wzx1210/innox2024_wc:v3.0.1 |  更新配置

## 环境说明

- Docker-Image: wzx1210/innox2024_wc:v1.0.1
- Ubuntu 20.04
- Ros Neotic 
- Mamba
- vscode-plugin: cmake / ros / c++ / python
- docker-network-mode: bridge

## 环境测试

|          | Linux | Windows                     | PI | Macos |
| -------- | ----- | --------------------------- | -- | ----- |
| 测试镜像 | wzx1210/innox2024_wc:v1.0.2      | wzx1210/innox2024_wc:v1.0.1 | registry.cn-hangzhou.aliyuncs.com/wzx1210/innox2024_wc:v3.0.1   |       |
| 可以启动 | √    | √                          |   √  |       |
| 可视化   | √     | √                          |  √   |       |
| 硬件通讯（USB_CAM） | √     | ×                          |  √   |       |
| 硬件通讯（下位机） |  √      |                           |  √   |       |

## workflow

如果你有兴趣可以阅读以下内容，也可以直接跳过

![Alt text](.config_res/docs/image.png)

1. 使用devcontainer作为开发环境。 https://containers.dev/
   1. devcontainer的一个特征是将代码的运行环境与代码本身完全隔离开来，在效果上类似将workspace挂载到了容器中。
   2. 在docker-image中仅仅包含有关系统的配置（例如修改baserc或安装依赖包等），其本身不存储任何项目代码和工作空间，做到了代码与环境的完全隔离
   3. 可以通过devcontainer.json配置文件，快速修改和分发容器配置。
   4. 与vscode深度融合，一键启动，无需任何命令

# OrangePi (营期使用)
1. 在你的电脑上安装vscode
   https://code.visualstudio.com/
2. 打开vscode,并安装remote SSH插件
   ![Alt text](6330b9f66bda98285e48e2ce6f883f8.png)
3. 远程连接Pi
   ![Alt text](37782d5f277e19f68e1bf8c0e6273ee.png)
4. 在Pi中克隆工作空间
   ```bash
   git clone https://github.com/RM-camp-for-high-school-students/Shenzhen_innoX_2024wc_ws.git
   ```
5. 在vscode中打开刚刚克隆的工作空间工作空间
   

# X86
## 环境准备

理论上支持任何可以安装以下准备环境的平台（arm平台未测试），如：Windows、Linux（如需连接硬件请使用linux）、MacOS

1. 安装docker
   1. 如果安装的是Docker Desktop，请在使用前启动Docker Engine
2. 安装vscode
3. 在vscode中安装devcontainer插件
   1. 建议保持版本一致 v0.299.0![image.png](https://cdn.nlark.com/yuque/0/2024/png/34306602/1705432899215-ce85fd14-8fca-469b-a669-61271b1e4ba2.png#averageHue=%232d3c47&clientId=ud3494556-2b48-4&from=paste&height=170&id=u74ceae26&originHeight=213&originWidth=1147&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=36225&status=done&style=none&taskId=uc9dc4c6a-b87d-4d22-b69b-89bd0cb0ba5&title=&width=917.6)

## 使用教程

1. 克隆目录：在**本地宿主机**的合适目录克隆代码

   ```bash
   git clone https://github.com/RM-camp-for-high-school-students/Shenzhen_innoX_2024wc_ws.git
   git checkout docker
   vscode .
   ```
2. 构建开发容器：在vscode中打开项目目录  -> 按 "Ctrl + Shift + P" 打开命令面板 -> 输入"Rebuild and Reopen in Container"构建开发容器。
   注意：
      1. 首次拉取镜像可能需要较长时间，你可以双击vscode右下角的通知消息查看当前log
      2. 请根据宿主机电脑的设置配置 .devcontainer.json 中的硬件设备，如配置错误则无法成功构建开发容器
   ![image.png](https://cdn.nlark.com/yuque/0/2024/png/34306602/1705433073032-015bca1c-9f9c-45db-b712-2552946ef5bc.png#averageHue=%233d474c&clientId=ud3494556-2b48-4&from=paste&height=172&id=u764cce2d&originHeight=215&originWidth=787&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=24361&status=done&style=none&taskId=uf427a904-ffa2-4792-aa1d-38cf0dc7b3c&title=&width=629.6)
3. 进入开发容器：等待镜像下载完成后，开发容器会自动构建并运行。此时左下角会显示【Shenzhen-innoX-2024wc】，表面已进入开发容器。
   ![image.png](https://cdn.nlark.com/yuque/0/2024/png/34306602/1705433378463-56672b9f-a39c-4708-8195-0eaad32a7b3b.png#averageHue=%232a3439&clientId=ud3494556-2b48-4&from=paste&height=1103&id=u9ed6cd66&originHeight=1379&originWidth=2559&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=173488&status=done&style=none&taskId=uaea158e0-808e-46ab-940e-df233a96143&title=&width=2047.2)
4. 编译工作空间：在vscode中的**开发容器终端**中运行以下命令

   ```bash
   catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
   source ~/.bashrc
   ```
5. 启动可视化：依次点击"端口选项卡" -> "desktop(6090)" 后的小地球 -> "输入密码vscode"，即可进入简易图形化系统界面
   ![image.png](https://cdn.nlark.com/yuque/0/2024/png/34306602/1705433966347-c61b7500-2497-42a6-8b7f-c686783ee7de.png#averageHue=%23cabe6d&clientId=ud3494556-2b48-4&from=paste&height=1058&id=u7a6487fb&originHeight=1322&originWidth=2214&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=102976&status=done&style=none&taskId=ucad6be92-51cc-458a-91f3-20510ab3603&title=&width=1771.2)
6. 测试可视化正常：运行以下命令，并观察是否可视化显示

   ```bash
   roslaunch map_server innox2024_wc.launch
   ```
   ![image.png](https://cdn.nlark.com/yuque/0/2024/png/34306602/1705434362352-925ec0bd-fb54-4fea-acaf-ee49c048df30.png#averageHue=%233b474f&clientId=ud3494556-2b48-4&from=paste&height=452&id=u6ea22e98&originHeight=1303&originWidth=2063&originalType=binary&ratio=1.25&rotation=0&showTitle=false&size=392216&status=done&style=none&taskId=uac21f97e-badc-4817-a729-c1d085647e8&title=&width=715)

## 下位机通讯


wzx1210

2023.1.18
