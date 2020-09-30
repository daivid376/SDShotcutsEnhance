author:daiwei 
w.dai@outlook.com
https://www.artstation.com/artwork/3oP8Bg

![01](https://cdnb.artstation.com/p/assets/images/images/019/767/601/original/david-leoric-01.gif)
Create new node at your mouse position ,using shortcuts

![02](https://cdnb.artstation.com/p/assets/images/images/019/767/651/original/david-leoric-02.gif)
The new node will be connected to the selected nodes automatically,when creating. At the same time, adjust postion to fit the selected nodes.



![01](https://cdna.artstation.com/p/assets/images/images/019/767/658/original/david-leoric-03.gif)
Disconnect a node from the other nodes, and try the best to reconnect previous input nodes to output nodes.
shortcut Shift + X

![01](https://cdnb.artstation.com/p/assets/images/images/019/767/757/original/david-leoric-04.gif)
Reconnect existing nodes.Could affect multi properties. and change connect order by the position of nodes.
shortcut C


![01](https://cdna.artstation.com/p/assets/images/images/019/767/480/original/david-leoric-05.gif)
Insert a node into some existing connections, when the node is acrossed by the connection.
Support multi connections .
shortcut C


![01](https://cdnb.artstation.com/p/assets/images/images/019/767/601/original/david-leoric-06.gif)
Rearrange selected nodes. Push them apart, when the nodes are two close.
shortcut Q


![01](https://cdnb.artstation.com/p/assets/images/images/019/767/601/original/david-leoric-07.gif)
Align nodes by vertical or horizontal, when the nodes are very limited offset by each other.
shortcut Q


![01](https://cdnb.artstation.com/p/assets/images/images/019/767/601/original/david-leoric-08.gif)
Rearrange input nodes mainly , when the input nodes have connected to some other nodes.
The nodes' new position will affected by the relative position of connected property.
shortcut Q


使用方法：
找到你的SD插件目录，一般在你的 [安装路径]\Substance Designer 2019\resources\python\sdplugins 这个位置
把整个文件夹SDShortcutsEnhance 解压到该路径下即可。

或者使用自定义的插件路径效果也是一样

Setup:
find your SD plugin folder,normally located at [application installed path]\Substance Designer 2019\resources\python\sdplugins
copy SDShortcutsEnhance folder to the plugin folder. done!
(use your custom plugin path is the same)

Notification:
 I can't get an  azerty  keyboad ,all shortcuts are tested and worked only on a qwerty keyboard.
This is a qwerty keyboard plugin only


Shortcuts:

specialFunctions

C	reconnect selected nodes
Q	rerange selected nodes
shift+X	disconnect selected nodes


compGraph

t	transform
b	blend over
a	blend add
shift+s      blend substract
shift+d     blend divide
m	blend multiplier
alt+s	slop blur
l 	level
w	warp
shift+w	directional warp
ctrl+w	multi directional warp
ctrl+shift+w	non uniform directional warp
u	uniformColor
shift+c	curve
g	gradient map
ctrl+h	hsl
shift+h	histgram scan
ctrl+b	blur
shift+b	blur hq
ctrl+shift+b 	non uniform blur
shift+a 	auto levle
ctrl+i	invert
p	pixel processor 16bit grayscale
shift+f	add frame

funcGraph
1	float1
2	float2
3	float3
4	float4	
shift+1	get float1
shift+2	get float2
u	get float2( input = '$pos', works as uv)
shift+3	get float3
ctrl+2 	vector float2
ctrl+3	vector float3
ctrl+4 	vector float4
alt+1	swizzle1
alt+2	swizzle2
alt+3	swizzle3
alt+4	swizzle4
a	add
shift+s	substract
shift+d	divide
m	multiplier
shift+m	scalar multiplier
p	power
i	if else
l	lerp
r	random
-	negation
g	sample grey
shift+c	sample color




