# tcpdump 抓取数据
```shell
sudo tcpdump -i enp4s0 host 192.168.4.4 and port 9760 -A | grep '"reqType":"photo"'
```

# 信息发送与返回示例
## 获取机械臂信息
```json
{"dsID":"www.hc-system.com.RemoteMonitor","reqType":"query","packID":"0","packID":"0","queryAddr":["axis-0","axis-1","axis-2","axis-3","axis-4","axis-5","curTorque-0","curTorque-1","curTorque-2","curTorque-3","curTorque-4","curTorque-5","curSpeed-0","curSpeed-1","curSpeed-2","curSpeed-3","curSpeed-4","curSpeed-5","world-0","world-1","world-2","world-3","world-4","world-5"]}
```
```json
{"dsID":"www.hc-system.com.RemoteMonitor","packID":"0","queryAddr":["axis-0","axis-1","axis-2","axis-3","axis-4","axis-5","curTorque-0","curTorque-1","curTorque-2","curTorque-3","curTorque-4","curTorque-5","curSpeed-0","curSpeed-1","curSpeed-2","curSpeed-3","curSpeed-4","curSpeed-5","world-0","world-1","world-2","world-3","world-4","world-5"],"queryData":["1385.070435","-128.817780","1578.943604","178.635025","14.756925","174.511032","1385.070435","-128.817780","1578.943604","178.635025","14.756925","174.511032","1385.070435","-128.817780","1578.943604","178.635025","14.756925","174.511032","1385.070435","-128.817780","1578.943604","178.635025","14.756925","174.511032"],"reqType":"query"}
```

## 视觉接收
```json
{"dsID":"www.hc-system.com.cam","reqType":"photo","camID":0}
```
```json
{"dsID":"www.hc-system.com.cam", "reqType":"photo","camID":0,"ret":1}
```

## 获取世界坐标系
```json
{"dsID":"www.hc-system.com.RemoteMonitor","reqType":"query","packID":"0","packID":"0","queryAddr":["world-0","world-1","world-2","world-3","world-4","world-5"]}
```
```json
{"dsID":"www.hc-system.com.RemoteMonitor","packID":"0","queryAddr":["world-0","world-1","world-2","world-3","world-4","world-5"],"queryData":["-137.563782","-2597.112793","1039.199219","-128.659561","89.213661","141.060074"],"reqType":"query"}
```

## 视觉指令
```json
{"dsID":"www.hc-system.com.cam", "reqType":"AddPoints", "dsData":[{"camID":"0", "data":[{"ModelID":"0","X":"-137.563782","Y":"-2597.112793","Z":"1039.199219","U":"-128.659561","V":"89.213661","Angel":"141.060074","Similarity":"0","Color":"0","Rel":"0"}]}]}
```
```json
{"cmdReply":["AddPoints","1"],"dsID":"www.hc-system.com.cam","reqType":"AddPoints"}
```
