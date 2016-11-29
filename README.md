#这个文档收录我读研期间所做的算法仿真

+ Distributed filter for two-target tracking mobile sensor network
	
		1. myfilter.py
		2. flocking.py
		3. bad_flocking.py
	
+ Event-triggered Kalman-consensus filter for two-target tracking sensor network

		1. event-triggered.py

+ 分数阶仿真问题
		
		1. yyySimulation.py
	
	这一个仿真应该是我做的最后的算法仿真工作了，目测也是我最好的一个，对于多智能体问题的封装我觉得已经到极限了，如果以后有新的算法只需要改动 agent::_update和agent::next函数就可以。而且用了Iterator的思想，算法迭代起来更方便。可惜我的师弟师妹们都不用python，目测它只能安安静静的躺在这里了。
	
目前所有的文件都是python2.7编译的，有时间希望把它们改到python3.5上面。也希望有时间可以能改成yield实现，感觉更cool。

哎~ 三年时间两篇论文几个仿真，虽然有些给别人写的仿真没有放上来，但是想想自己做的实验室工作真的不算太多，嘛把代码贴上来论文也投出去了 算是对自己研究生生涯的总结吧。

我还是很不喜欢这个实验室，幸运的是现在终于可以往前看了。	
	