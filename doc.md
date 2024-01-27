1. 连接wifi到Innoxsz-Public 密码 innox2023


2. orangepi上电，typec口插竖的typeC口，如图所示，红色方框标出的的口
![](https://cdn.studyinglover.com/pic/2024/01/b5003c2040f429e61e88f70b684ce4ba.jpg)


3. 安装nomachine
    官网链接 https://downloads.nomachine.com/

4. 使用nomachine连接到树莓派
    1. 打开nomachine
    ![](https://cdn.studyinglover.com/pic/2024/01/46a63892036423f784c9df8187e8ce05.png)
    2. 上图我们可以看到已经自动找到了一些设备，没找或者什么都没有都没关系，我们点击Add按钮
    ![](https://cdn.studyinglover.com/pic/2024/01/c6a0599a65f55d9c6a20ddf172b02dae.png)
     
     写入你的orangepi的ip,然后点击Add
     ![](https://cdn.studyinglover.com/pic/2024/01/4a3ccbdecc28bec4288edc6b1a82ab45.png)

     我们可以看到这里已经有了我们的设备
     ![](https://cdn.studyinglover.com/pic/2024/01/509b11ff3b964f3bfc9707d67319f2e6.png)

     双击，连接，按照下图指引输入账号密码(账号密码都是`orangepi`)，选择保存密码，点击OK
     ![](https://cdn.studyinglover.com/pic/2024/01/0915c3de27d200a905436d861f026cc0.png)

     依然选择OK，一直点击OK,一切顺利的话你会看到这样的界面
     ![](https://cdn.studyinglover.com/pic/2024/01/c6d15f1c538d1e30657dee0a97c380e2.png)

5. 使用vscode连接到orangepi
    1. 下载插件remote, 点击左侧的插件，输入remote,回车，选择Remote-SSH，点击安装
    ![](https://cdn.studyinglover.com/pic/2024/01/10902aae9bea5233fa541fae417c4f27.png)
    2. 安装成功后，你会在vscode左侧出现这样一个图标，点击打开
    ![](https://cdn.studyinglover.com/pic/2024/01/3ca3f3c97b99e006e9438b75482fb0cb.png)
    3. 打开后将鼠标移动到SSH这一行，点击 右侧的加号，新建远程
    ![](https://cdn.studyinglover.com/pic/2024/01/14995afd2a1c00f718ae3ea64c8f48db.png)

    4. 在上方会出现这样的一个界面，输入`ssh orangepi@<ip>`, 将`<ip>`换成你的orangepi的ip(例如`ssh orangepi@10.10.42.231`)，然后回车
    ![](https://cdn.studyinglover.com/pic/2024/01/91f2ae53b85a320e670f2d7bc9fc51fb.png)

    5. 选择第一个选项
    ![](https://cdn.studyinglover.com/pic/2024/01/5f21bde7d8ad405d086a5adeba869bf5.png)

    6. 在左侧的界面点击刷新一下
    ![](https://cdn.studyinglover.com/pic/2024/01/25c92413b2006ab15117fe571eb7fce3.png)

    7. 可以看到你的orangepi已经出现在这里了，点击连接，出现一个输入密码的窗口，输入密码(密码是`orangepi`)，回车
    ![](https://cdn.studyinglover.com/pic/2024/01/8cd83f97fcceb352539f51905c30bea7.png)

    8. 右下角会显示正在下载vscode服务器，请稍作等待，如果一直卡住不动回到第1步重新安装Remote-SSH, 可以选择换一个版本安装
    ![](https://cdn.studyinglover.com/pic/2024/01/e46a58f7f212b4415a23a3cdd73f2c51.png)

    9. 当左下角出现这个图标的时候，说明连接成功了，ip应该是你的orangepi ip
    ![](https://cdn.studyinglover.com/pic/2024/01/9e633f00f6ca7491bf81c706ffb74903.png)

6. clone仓库
    1. 打开vscode的终端，点击左侧的终端，选择新建终端
    ![](https://cdn.studyinglover.com/pic/2024/01/34004fb10a6a36f4fed7fdbca31c9d81.png)
    2. 下方会出现一个终端，输入`git clone https://github.com/RM-camp-for-high-school-students/Shenzhen_innoX_2024wc_ws`，回车
    ![](https://cdn.studyinglover.com/pic/2024/01/f5eb2786d47242e52066f963c9f9fc1e.png)
    3. 当你看到这样的界面的时候，说明clone成功了
    ![](https://cdn.studyinglover.com/pic/2024/01/1f7ae5e58a27da91453d09acf408ff8b.png)

    4. 点击界面上的 `打开文件夹` 
    ![](https://cdn.studyinglover.com/pic/2024/01/32ab72b3e2c33816cd926ba43fb16b0a.png)

    5. 向下滑动，选择`Shenzhen_innoX_2024wc_ws`
    ![](https://cdn.studyinglover.com/pic/2024/01/84aca26b6425e20ab02647f9caa9e316.png)

    6. 点击确定
    ![](https://cdn.studyinglover.com/pic/2024/01/144a86a4d01ac5b700a2ad436276fcf7.png)
    
    7. 你会看到这样的界面，说明你已经成功打开了仓库
    ![](https://cdn.studyinglover.com/pic/2024/01/148273130a7a724b9a4fb9bffa4f1fab.png)

7. 构建开发环境
    1. 在已经打开了远程orangepi的vscode中安装dev container插件，点击左侧的插件，输入remote,回车，选择Dev Containers，点击安装
    ![](https://cdn.studyinglover.com/pic/2024/01/200580da548fc6c52b8b4bfdddf66871.png)

    2. 让我们解释一下，我们的开发环境是构建在docker上面的，配置docker的文件是在`.devcontainer/devcontainer.json` ，我们对这个文件一些可能需要修改的东西做一些解释

    在我们的配置文件中有这么几行，这几行是挂载你们的物理设备到docker镜像中，所有如果你们的C板没有插到orangepi上面就注释掉`"--device=/dev/ttyACM0:/dev/robomaster",`这一行，如果没有摄像头就注释掉`"--device=/dev/video0:/dev/video0",` 这一行。
    ![](https://cdn.studyinglover.com/pic/2024/01/f8e9ffb381ce1d214db2f8860c379b1e.png)

    3. 打开命令面板，点击左下角的齿轮按钮，点击命令面板，或者快捷键`Ctrl + Shift + P`
    ![](https://cdn.studyinglover.com/pic/2024/01/2a72c56684009dd24b658653bf37b225.png)

    4. 输入`rebuild`,选择 `开发容器，在容器中重新生成并重新打开`
    ![](https://cdn.studyinglover.com/pic/2024/01/585076c52978d88c9d2f1e59ae7e4187.png)

    5. 稍作等待，右下角会有这样的弹窗，你可以双击查看日志
    ![](https://cdn.studyinglover.com/pic/2024/01/89a749ed8bde291d86bc90231c8923b5.png)

    6. 出现这个，点一下旁边或者按一下`Esc`
    ![](https://cdn.studyinglover.com/pic/2024/01/4fd9c75e50c5484988c127ccb46a7b95.png)

    6. 加载完成后，你可以看到右下角变成了`Dev Container: Shenzhen_innoX_2024wc_ws`，说明你已经成功进入了docker容器
    ![](https://cdn.studyinglover.com/pic/2024/01/739953a8ff6ba4db25b29ac6f27b759e.png)

    ![](https://cdn.studyinglover.com/pic/2024/01/8cdca9185860a6784547b8d54d8e839d.png)

    7. 你可以在现在的环境中进行开发了