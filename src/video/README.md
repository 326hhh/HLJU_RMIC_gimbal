# gimbal/src/video —— MJPEG 图传（独立程序）

本目录只有 `mjpeg_server.cpp`，编译成**独立可执行文件** `build/mjpeg_server`（不在主控制循环里）。
职责：从车载 UVC 摄像头（V4L2）取 MJPG 帧 → 经 HTTP 转发给 PC 客户端显示。

> 为什么零转码：UVC 摄像头本身能输出 MJPG（压缩帧），V4L2 拿到的每个 buffer 就是一张完整
> JPEG（`FFD8…FFD9`），原样转发即可，CPU 几乎不花（Jetson 上实测 <3%）。

## 文件一览

| 文件 | 内容 |
|---|---|
| `mjpeg_server.cpp` | `V4l2Capture`（采集）+ `MjpegServer`（HTTP 转发）+ `main` |

---

## 1. 全局部分

| 成员 | 行为 |
|---|---|
| `g_run` / `onSignal` | SIGINT/SIGTERM 退出标志；`SIGPIPE` 忽略（客户端断连不杀进程） |
| `nowS()` | 单调时钟秒数（统计用） |
| `usage()` | 打印帮助（`--device/--size/--fps/--port`） |

## 2. `class V4l2Capture` —— V4L2 MJPG 采集

| 函数 | 行为 |
|---|---|
| `open(dev, w, h, fps)` | `VIDIOC_QUERYCAP` → `VIDIOC_S_FMT` 请求 MJPG（**校验实际返回的 pixelformat 必须是 MJPG**，否则报错退出）→ `VIDIOC_G/S_PARM` 设帧率 → `VIDIOC_REQBUFS` 申请 4 个 MMAP 缓冲 → 逐个 `VIDIOC_QUERYBUF` + `mmap` + `VIDIOC_QBUF` → `VIDIOC_STREAMON` |
| `dequeue(&data,&len,&idx)` | 非阻塞 `VIDIOC_DQBUF`；`EAGAIN` 时返回 false；成功给出数据指针/长度/索引 |
| `requeue(idx)` | `VIDIOC_QBUF` 把缓冲还给驱动（处理完必须调） |
| `width()/height()` | 实际分辨率（可能是摄像头协商后的值） |
| `close()` | `STREAMOFF` → 逐个 `munmap` → `close` |
| `~V4l2Capture()` | `close()` |

细节：`kBufCount = 4`；`VIDIOC_REQBUFS` 返回缓冲数 <2 直接报错；确保流停掉再释放内存。

## 3. `class MjpegServer` —— 极简 MJPEG HTTP 服务器

| 函数 | 行为 |
|---|---|
| `listen(port)` | `socket(SOCK_STREAM\|SOCK_NONBLOCK\|SOCK_CLOEXEC)` → `SO_REUSEADDR` → `bind(0.0.0.0:port)` → `listen(4)` |
| `acceptClients()` | `accept4()` 非阻塞循环；超过 `kMaxClients=4` 直接关掉新连接；`TCP_NODELAY`；读掉请求行（不解析路径，一律给流）；发送 multipart 响应头 |
| `broadcast(jpeg, len)` | 对每个客户端发送 `--ulinkframe` 分段头 + JPEG 数据；任一段发送失败（慢客户端/断开）→ 关闭并从列表移除 |
| `clientCount()` | 当前客户端数（状态打印用） |
| `close()` | 关客户端与监听 socket |
| `~MjpegServer()` | `close()` |

## 4. `main()` 主循环

```cpp
V4l2Capture cap; cap.open(...);           // 失败直接退出
MjpegServer srv; srv.listen(port);
while (g_run) {
    srv.acceptClients();
    拿一帧（dequeue）；没有帧 → sleep 5ms 重试（避免空转吃满 CPU）；
    srv.broadcast(data, len); cap.requeue(idx);
    每秒打印统计：分辨率、fps、KB/s、客户端数、最大帧间隔
}
退出：srv.close(); cap.close();
```

## 5. HTTP 协议细节

- 响应头：`Content-Type: multipart/x-mixed-replace; boundary=--ulinkframe`；
- 每帧边界：`--ulinkframe\r\nContent-Type: image/jpeg\r\nContent-Length: <n>\r\n\r\n` + JPEG；
- 客户端 `~/gimbal_ctl` 的 `MjpegView` 是**按 JPEG 的 SOI(FFD8)/EOI(FFD9) 切帧**的，
  不依赖 boundary 文本 —— boundary 只是给浏览器（`http://<ip>:8080/stream`）看的。

## 6. 带宽参考（MJPG，动态场景）

| 分辨率@帧率 | 带宽 |
|---|---|
| 640×480@30 | ≈ 3~6 Mbps |
| 800×600@20 | ≈ 3~5 Mbps |
| 1280×720@30 | ≈ 8~15 Mbps |

Jetson AP（2.4G）实测能到 20+ Mbps，建议 640×480 或 800×600 起步。
低光下自动曝光会拉长到 1/15s，帧率掉到 ~16-20fps 属正常现象。

## 7. 注意

- 本程序只接受 **MJPG** 格式的摄像头；只支持 YUYV 的摄像头会直接报错（不做转码）；
- 是独立进程，与云台控制解耦：`./build/mjpeg_server` 与 `./build/gimbal` 各跑各的；
- 慢客户端会被直接踢掉（不会拖慢其他客户端）。
