# Example Hello World Capability

This is an example capability that simply prints "Hello, World!" to the file.



Test CMD as follow:


```shell
# 启动hello node 节点
python ./hello_src.py

# 查看hello node节点
ros2 node list # /hello_node
ros2 node info /hello_node
# 查看服务
ros2 service list # /get_count /modify_name /shutdown_node ...
ros2 service type /get_count # std_srvs/srv/Trigger

# hello_api.py测试服务是否运行，将mcp.run注释并启用test
python hello_api.py 
```

TODO:实时监测mcptool是否上线-随时更新

设计一个整体的接口

并行优化-将下一个操作动作的初始化提前
