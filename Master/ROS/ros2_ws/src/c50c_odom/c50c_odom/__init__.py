"""C50C 平面里程计候选积分器(候选代码,离线验证版)。

纯 Python、无 ROS 依赖的独立包;ROS 侧经 ``c50c_uart3.odom_msg`` 消费。
实现见 :mod:`c50c_odom.odometry`,正确性边界与未验证项见该模块文档及
``task-results/c50c-odom-prep.md``。**状态:候选——不发布 /odom,
不发 TF,不接建图节点。**
"""
