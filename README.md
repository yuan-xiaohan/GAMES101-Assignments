# Homework2
GAMES101 Homework2

## 完成部分：共50分，完成了提高部分

•   [5 分]  正确地提交所有必须的文件，且代码能够编译运行。

•   [20 分] 正确实现三角形栅格化算法。

•   [10 分] 正确测试点是否在三角形内。

•   [10 分] 正确实现 z-buffer 算法, 将三角形按顺序画在屏幕上。

•   [提高项 5 分] 用 super-sampling 处理 Anti-aliasing。

## 函数功能：
•   rasterize_triangle(): 执行三角形栅格化算法。

•   static bool insideTriangle(): 测试点是否在三角形内。你可以修改此函 数的定义，这意味着，你可以按照自己的方式更新返回类型或函数参数。

•   增加了两个buffer：depth_buf_ss和depth_buf_ss用来super-sampling。

•   MSAA(): 执行反走样算法。

•   get_index_ss(): 获得新的index。
