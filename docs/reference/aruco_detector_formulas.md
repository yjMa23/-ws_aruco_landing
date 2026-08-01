# ArUco 检测模块公式说明

本文档对应 [`aruco_detector_node.cpp`](../../src/aruco_detector/src/aruco_detector_node.cpp)，只解释当前实现实际使用或直接依赖的数学关系。OpenCV 内部的候选轮廓提取、角点细化和 PnP 求解器实现不在本项目中重复展开。

## 1. 处理链路

每次近似时间同步得到一对 `Image` 和 `CameraInfo` 后，节点依次执行：

$$
\text{BGR 图像}
\rightarrow \text{灰度图}
\rightarrow \text{ArUco 角点与 ID}
\rightarrow \text{目标 ID 筛选}
\rightarrow \text{PnP 位姿}
\rightarrow (\mathbf{t},\mathbf{q})
$$

其中：

| 符号 | 含义 | 代码对应 |
| --- | --- | --- |
| $L$ | Marker 实际边长，单位 m | `marker_length_` |
| $(u_i,v_i)$ | 第 $i$ 个角点的像素坐标 | `target_corners` |
| $\mathbf{K}$ | 相机内参矩阵 | `camera_matrix` |
| $\mathbf{d}$ | 镜头畸变系数 | `dist_coeffs` |
| $\mathbf{r}$ | OpenCV 旋转向量 | `rvecs.front()` |
| $\mathbf{R}$ | Marker 到相机的旋转矩阵 | `rotation_matrix` |
| $\mathbf{t}$ | Marker 中心在相机坐标系的位置 | `tvecs.front()` |
| $\mathbf{q}$ | ROS 四元数，消息顺序为 $(x,y,z,w)$ | `pose.orientation` |

## 2. 灰度变换与 ID 匹配

`cv::cvtColor(..., cv::COLOR_BGR2GRAY)` 使用标准亮度加权关系，可写为：

$$
Y \approx 0.114B + 0.587G + 0.299R
$$

OpenCV 随后从灰度图中寻找方形候选、透视展开内部编码，并与配置的字典比较。二进制编码匹配可用汉明距离表示：

$$
d_H(\mathbf{b},\mathbf{c})=
\sum_{j=1}^{n}\mathbf{1}(b_j\neq c_j)
$$

实际阈值、纠错和候选筛选由 `cv::aruco::detectMarkers()` 负责。项目代码只查找满足

$$
id_i=target\_id
$$

的第一个检测结果，其他 ID 只画在调试图像上，不估计和发布位姿。

## 3. 相机模型

### 3.1 内参矩阵

代码直接按行复制 `CameraInfo.k` 的九个元素：

$$
\mathbf{K}=
\begin{bmatrix}
k_0 & k_1 & k_2\\
k_3 & k_4 & k_5\\
k_6 & k_7 & k_8
\end{bmatrix}
$$

常规针孔相机标定结果通常为：

$$
\mathbf{K}=
\begin{bmatrix}
f_x & s & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
$$

其中 $f_x,f_y$ 是像素焦距，$(c_x,c_y)$ 是主点，$s$ 是 skew。当前有效性检查仅为：

$$
k_0>0,\qquad k_4>0,\qquad k_8\neq 0
$$

若不满足，节点发布 `visible=false`，跳过 PnP。

### 3.2 针孔投影与畸变

相机坐标系中一点 $\mathbf{P}_c=(X_c,Y_c,Z_c)^T$ 的归一化坐标为：

$$
x=\frac{X_c}{Z_c},\qquad y=\frac{Y_c}{Z_c}
$$

对于常见的 `plumb_bob` 前五项畸变
$\mathbf{d}=(k_1,k_2,p_1,p_2,k_3)$：

$$
r^2=x^2+y^2
$$

$$
\alpha=1+k_1r^2+k_2r^4+k_3r^6
$$

$$
x_d=x\alpha+2p_1xy+p_2(r^2+2x^2)
$$

$$
y_d=y\alpha+p_1(r^2+2y^2)+2p_2xy
$$

最终像素满足：

$$
\lambda
\begin{bmatrix}
u\\v\\1
\end{bmatrix}
=
\mathbf{K}
\begin{bmatrix}
x_d\\y_d\\1
\end{bmatrix}
$$

`CameraInfo.d` 为空时，代码显式使用五个零，即无畸变；不为空时直接整行传给 OpenCV。代码不检查 `distortion_model`，更多系数的解释由当前 OpenCV 标定模型决定。

## 4. Marker 三维点与 PnP

OpenCV 以 Marker 中心为原点。默认中心坐标约定下，边长为 $L$ 的四个平面角点可写为：

$$
\mathbf{P}_1^m=\left(-\frac L2,\frac L2,0\right),\quad
\mathbf{P}_2^m=\left( \frac L2,\frac L2,0\right)
$$

$$
\mathbf{P}_3^m=\left( \frac L2,-\frac L2,0\right),\quad
\mathbf{P}_4^m=\left(-\frac L2,-\frac L2,0\right)
$$

Marker 点到相机坐标系的刚体变换为：

$$
\mathbf{P}_i^c=\mathbf{R}\mathbf{P}_i^m+\mathbf{t}
$$

PnP 根据四组 3D-2D 对应关系估计 $\mathbf{R},\mathbf{t}$。其目标可概括为最小化重投影误差：

$$
(\mathbf{R}^*,\mathbf{t}^*)=
\arg\min_{\mathbf{R},\mathbf{t}}
\sum_{i=1}^{4}
\left\|
\mathbf{p}_i-
\pi(\mathbf{K},\mathbf{d},\mathbf{R}\mathbf{P}_i^m+\mathbf{t})
\right\|_2^2
$$

$\pi(\cdot)$ 表示带畸变的相机投影，$\mathbf{p}_i=(u_i,v_i)^T$ 是检测角点。`estimatePoseSingleMarkers()` 返回：

- `tvec`：$\mathbf{t}$，即 Marker 中心在相机坐标系中的位置；
- `rvec`：$\mathbf{r}$，即 $\mathbf{R}$ 的轴角形式。

平移的尺度由 $L$ 决定。若边长参数整体放大为 $\beta L$，同一图像下估计的平移尺度也近似变为 $\beta\mathbf{t}$，因此 `marker_length` 必须使用真实米制尺寸。

## 5. Rodrigues 旋转向量

令：

$$
\theta=\|\mathbf{r}\|_2,\qquad
\mathbf{k}=\frac{\mathbf{r}}{\theta}
$$

$[\mathbf{k}]_\times$ 为叉乘矩阵：

$$
[\mathbf{k}]_\times=
\begin{bmatrix}
0 & -k_z & k_y\\
k_z & 0 & -k_x\\
-k_y & k_x & 0
\end{bmatrix}
$$

`cv::Rodrigues()` 实现的关系是：

$$
\mathbf{R}=\mathbf{I}
+\sin\theta[\mathbf{k}]_\times
+(1-\cos\theta)[\mathbf{k}]_\times^2
$$

当 $\theta\rightarrow0$ 时，$\mathbf{R}\rightarrow\mathbf{I}$。这里的 $\mathbf{R}$ 与 $\mathbf{t}$ 一起描述 Marker 坐标系到相机坐标系的变换。

## 6. 旋转矩阵转四元数

设 $\mathbf{R}=[m_{ij}]$，迹为：

$$
\tau=m_{00}+m_{11}+m_{22}
$$

代码按下表从上到下选择第一个满足的分支，以使用数值较稳定的矩阵元素。表中的四元数顺序统一写成 $(w,x,y,z)$；写入 ROS 消息时对应字段仍是 `x/y/z/w`。

| 条件 | $s$ | $(w,x,y,z)$ |
| --- | --- | --- |
| $\tau>0$ | $2\sqrt{\tau+1}$ | $\left(\frac{s}{4},\frac{m_{21}-m_{12}}s,\frac{m_{02}-m_{20}}s,\frac{m_{10}-m_{01}}s\right)$ |
| $m_{00}>m_{11}$ 且 $m_{00}>m_{22}$ | $2\sqrt{1+m_{00}-m_{11}-m_{22}}$ | $\left(\frac{m_{21}-m_{12}}s,\frac{s}{4},\frac{m_{01}+m_{10}}s,\frac{m_{02}+m_{20}}s\right)$ |
| $m_{11}>m_{22}$ | $2\sqrt{1+m_{11}-m_{00}-m_{22}}$ | $\left(\frac{m_{02}-m_{20}}s,\frac{m_{01}+m_{10}}s,\frac{s}{4},\frac{m_{12}+m_{21}}s\right)$ |
| 其他 | $2\sqrt{1+m_{22}-m_{00}-m_{11}}$ | $\left(\frac{m_{10}-m_{01}}s,\frac{m_{02}+m_{20}}s,\frac{m_{12}+m_{21}}s,\frac{s}{4}\right)$ |

随后统一归一化：

$$
n_q=\sqrt{x^2+y^2+z^2+w^2},\qquad
\mathbf{q}\leftarrow\frac{\mathbf{q}}{n_q}
$$

若 $n_q=0$，代码回退为单位四元数 $(x,y,z,w)=(0,0,0,1)$。

## 7. Pose 发布关系

最终 `PoseStamped` 满足：

$$
pose.position=(t_x,t_y,t_z)
$$

$$
pose.orientation=(q_x,q_y,q_z,q_w)
$$

消息 `header` 完整复用输入图像的 `header`，因此 `frame_id` 仍应指向相机光学坐标系。节点本身不做 TF 变换。

相机光学坐标含义为：

- $t_x>0$：Marker 在图像右侧；
- $t_y>0$：Marker 在图像下方；
- $t_z>0$：Marker 位于镜头前方。

例如 $\mathbf{t}=(0.10,-0.05,3.00)$ m 表示 Marker 位于相机右侧 10 cm、上方 5 cm、前方 3 m。若 $\mathbf{R}=\mathbf{I}$，发布四元数为 $(0,0,0,1)$。

## 8. 有效位姿与失败路径

一次回调发布 `visible=true` 的条件可以写为：

$$
visible=
target\_found
\land valid(\mathbf{K})
\land (rvecs\neq\varnothing)
\land (tvecs\neq\varnothing)
$$

| 条件 | `/aruco/visible` | `/aruco/pose` | 调试图像 |
| --- | --- | --- | --- |
| 图像转换失败 | `false` | 不发布 | 不发布 |
| 目标 ID 不存在 | `false` | 不发布 | 发布检测框图像 |
| 内参无效 | `false` | 不发布 | 发布检测框图像 |
| PnP 结果为空 | `false` | 不发布 | 发布检测框图像 |
| PnP 成功 | `true` | 发布 | 发布检测框和坐标轴 |

`visible=false` 不会清除上一条 `/aruco/pose`，所以控制器还会额外检查可见性和接收时间。
