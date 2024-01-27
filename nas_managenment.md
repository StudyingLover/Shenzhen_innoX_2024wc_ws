我们的数据存储在2024winter_camp这个共享文件夹
![](https://cdn.studyinglover.com/pic/2024/01/297a0754cf7131dc975fa0aa7e89d8fd.png)

有两个组，分别是助教和学生

2024winter_camp下面有两个文件夹，分别是zhujiao和xuesheng

助教可以对zhujiao文件夹进行读写操作，对xuesheng文件夹进行读写操作
学生可以对xuesheng文件夹进行读写操作，对zhujiao文件夹进行只读操作

|组|文件夹|权限|
|-|-|-|
|zhujiao|zhujiao|读写|
|zhujiao|xuesheng|读写|
|xuesheng|xuesheng|读写|
|xuesheng|zhujiao|只读|

