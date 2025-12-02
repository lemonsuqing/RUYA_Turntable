TurntableController.exe --config="RUYAconfig.json" --command "Position Run" --acc 100 --speed 100 --angle 36000

###################Param####################
--config        string      转台配置文件     
.eg "RUYAconfig.json"
.eg "AC1120Sconfig.json"

--command       string      转台指令
.eg "Init"  
.eg "Position Run"  
.eg "Speed Run"  
.eg "Stop"  
.eg "Free Mode" 

--acc           Float       旋转加速度(deg/s^2)
.eg 100 

--speed         Float       旋转速度(deg/s)
.eg 100 

--angle         Float       旋转角度(deg)
.eg 36000

--printScreen   bool        转台数据输出到屏幕
.eg True  
.eg False

--SaveCSVFile   string      保存转台数据到CSV文件
.eg "*.csv"

##################Command###################
Init    初始化
--说明：
初始化转盘，转盘上电后，需要使能或进入伺服模式等操作才可以进行转动控制
例如：
国外转盘需要发 EN ，上锁转盘， OPMODE 0 ，进入速度模式等操作
国内转盘需要发 mo=1， 启用电机，使转台进入伺服状态等操作
--print
OK / Error        发送指令收到正确响应返回OK / 发送指令未收到正确响应返回Error
.eg
$ TurntableController.exe --config="RUYAconfig.json" --command "Init"
> OK

Position Run      位置模式运行
--说明：
位置模式下运行
--print
OK / Error          发送指令收到正确响应返回OK / 发送指令未收到正确响应返回Error
POSHEAD * / Error   旋转前，打印起始角度(deg) / 发送指令未收到角度数据返回Error
Complete / Error    旋转完成后检查转盘停下 / 多次确认转盘没有按输入指令按时停下返回Error
POSTAIL * / Error   旋转后，打印结尾角度(deg) / 发送指令未收到角度数据返回Error
.eg
$ TurntableController.exe --config="RUYAconfig.json" --command "Position Run" --acc 100 --speed 100 --angle 36000
> OK
> POSHEAD 0
> Complete 
> POSTAIL 0.03

$ TurntableController.exe --config="RUYAconfig.json" --command "Position Run" --acc 100 --speed 100 --angle 36000
> OK
> Error get data failed


Speed Run         速度模式运行
--说明：
速度模式下运行
--print
OK / Error        发送指令收到正确响应返回OK / 发送指令未收到正确响应返回Error
.eg
$ TurntableController.exe --config="RUYAconfig.json" --command "Speed Run" --acc 100  --speed 100 
> OK

Stop              停下转盘
--说明：
停下转盘
--print
OK / Error        发送指令收到正确响应返回OK / 发送指令未收到正确响应返回Error
.eg
$ TurntableController.exe --config="RUYAconfig.json" --command "Stop"
> OK

Free Mode         空闲模式
--说明：
释放转盘或进入空闲模式
例如：
国外转盘需要发 DIS ，解锁转盘
国内转盘需要发 mo=0， 释放电机，使转台进入空闲状态
--print
OK / Error        发送指令收到正确响应返回OK / 发送指令未收到正确响应返回Error
.eg
$ TurntableController.exe --config="RUYAconfig.json" --command "Free Mode"
> OK

