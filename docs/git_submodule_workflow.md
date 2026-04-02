# Git Submodule Workflow

## 适用场景

当前工作区采用的是：

- 父仓库：`/home/dianhua/Robot24_catkin_ws/src`
- 子仓库：`can_driver`
- 子仓库：`Eyou_Canopen_Master`
- 子仓库：`realsense-ros`

这意味着：

- 子仓库负责保存各自的完整开发历史
- 父仓库负责记录整个工作区在某个时刻引用了哪些子仓库版本
- 父仓库不会替代子仓库的历史，只会记录子仓库当前指向的 commit

## 一次性拉取整个工作区

首次克隆父仓库时，推荐：

```bash
git clone --recurse-submodules <parent-repo-url>
```

如果已经先普通 clone 了父仓库，再补拉子仓库：

```bash
git submodule update --init --recursive
```

如果之后父仓库里的子模块指针更新了，想同步到最新记录：

```bash
git pull
git submodule update --init --recursive
```

## 日常开发流程

### 1. 在子仓库里开发

以 `can_driver` 为例：

```bash
cd /home/dianhua/Robot24_catkin_ws/src/can_driver
git status
git add .
git commit -m "Fix can driver logic"
git push
```

### 2. 回父仓库记录子仓库版本变化

```bash
cd /home/dianhua/Robot24_catkin_ws/src
git status
git add can_driver
git commit -m "Update can_driver submodule"
git push
```

如果同时还改了父仓库自己的文件，比如 `.gitmodules`、`README.md`、`docs/` 下文档，也一起加上：

```bash
git add can_driver .gitmodules README.md docs/
git commit -m "Update workspace submodules"
git push
```

## 最常用判断方法

查看子仓库自己有没有代码改动：

```bash
git -C /home/dianhua/Robot24_catkin_ws/src/can_driver status
```

查看父仓库有没有子模块版本变化需要记录：

```bash
git -C /home/dianhua/Robot24_catkin_ws/src status
```

查看父仓库当前记录的子模块版本：

```bash
git -C /home/dianhua/Robot24_catkin_ws/src submodule status
```

## 能不能只提子仓库，晚点再提父仓库

可以。

常见情况是：

- 子仓库平时频繁提交
- 父仓库只在阶段稳定、联调通过、需要同步整套工作区时再提交一次

例如子仓库 `can_driver` 连续提交了：

- `A`
- `B`
- `C`

如果父仓库只在最后提交一次，那么父仓库只会记录：

- 之前引用的是旧提交
- 现在更新到了 `C`

父仓库不会逐条记录 `A -> B -> C` 的每一步变化。  
这些完整历史都还在 `can_driver` 子仓库里。

## 什么时候一定要补提父仓库

建议在这些时机补一次父仓库提交：

- 整个工作区编译通过了
- 联调通过了
- 准备切换机器开发
- 准备让别人拉你的最新工作区
- 准备打 tag 或留里程碑版本

原因是：别人从父仓库拉代码时，拿到的是父仓库记录的子模块版本，而不是你子仓库里“最新但尚未同步到父仓库”的版本。

## 一个完整例子

```bash
cd /home/dianhua/Robot24_catkin_ws/src/can_driver
git add .
git commit -m "Add zero-limit handling"
git push

cd /home/dianhua/Robot24_catkin_ws/src
git add can_driver
git commit -m "Update can_driver to tested revision"
git push
```

## 推荐理解方式

可以把两层仓库理解成：

- 子仓库：保存功能开发历史
- 父仓库：保存整机工作区的版本快照

一句话记忆：

> 子仓库负责开发，父仓库负责集成。
