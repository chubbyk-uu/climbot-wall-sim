# climbot_common

纯 Python、无 ROS 运行时依赖的跨包工具与共享中间件配置。仿真、巡检和离线拼接包都向它
依赖，方向不倒置：
`climbot_mosaic` 只需要 numpy/opencv/scipy/tifffile，不能为了一个 Git 助手把 `rclpy`
拖进来；`climbot_gazebo` 也不该为了同一个助手去依赖图像处理栈。

## 内容

| 模块 | 用途 |
| --- | --- |
| `provenance.git_state()` | 记录 commit、branch、`source_modified`、`checked_pathspecs` 与 `traceable` |
| `hashing.sha256_file()` / `sha256_bytes()` | 把一个阶段的输出哈希和下一个阶段的输入哈希统一成同一个定义 |
| `atomic.write_json()` / `json_text()` | 摘要要么整份发布要么不发布，且渲染形式逐字节固定 |
| `config/fastdds_inspection_image.xml` | 为 6.22 MB / 2.07 MB 可靠图像发布者预留 64 MiB 共享内存段 |

## 为什么 `checked_pathspecs` 是返回值的一部分

`source_modified` 单独一个布尔值说不出它检查了什么。原实现只看 `src`，而正式摘要生成器
在 `tools/` 下运行——生成器自己有未提交修改时，它照样报告干净树。默认 pathspec 因此是
`('src', 'tools')`，并把实际检查过的范围一起写进结果，让读者能区分"干净"和"没查"。

`--porcelain` 默认包含未跟踪文件，所以这些路径下新增的未提交源码同样算作已修改。

## 尚未迁移的重复实现

以下位置仍有各自的哈希或 Git 助手，迁移时优先合并到本包，不要再复制第四份：

- `src/climbot_inspection/climbot_inspection/archive_core.py`：`sha256_file` / `sha256_bytes`；
- `src/climbot_mosaic/climbot_mosaic/{diagnostic_truth,fusion,mosaic_inputs}.py`：三份 `_sha256`；
- `tools/{bake_wall_texture,resample_wall_texture,create_diagnostic_wall}.py`：只记 commit、
  不记 dirty 状态的 `git_commit()` 与 `sha256()`。
