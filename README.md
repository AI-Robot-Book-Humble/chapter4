# 第４章　ナビゲーション（改訂第2版）
## 概要
ROS 2とPythonで作って学ぶAIロボット入門 改訂第2版（出村・萩原・升谷・タン著，講談社）第４章のサンプルプログラムと補足情報などを掲載しています．

## ディレクトリ構成
- **[amcl_subscriber](amcl_subscriber)**: /amclと/odomをサブスクライブするパッケージ
- **[happy_lidar](happy_lidar)**: シンプルなLidarパッケージとドアオープンのサンプル
- **[happy_move](happy_move)**: シンプルな移動パッケージ
- **[happy_teleop](happy_teleop)**: 自作遠隔操作パッケージ
- **[map](map)**: 地図ファイルを格納するディレクトリ
- **[path_planning](path_planning)**: 経路計画のサンプルプログラム
- **[waypoint_navi](waypoint_navi)**: ウェイポイントナビゲーションのサンプルパッケージ

## インストール
Chapter4の全パッケージを以下のコマンドでインストールします．
- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- Chapter4のリポジトリを入手
  ```
  git clone https://github.com/AI-Robot-Book-Humble/chapter4
  ```

## 補足情報
- ナビゲーション関連のサンプルプログラムを実行するときは，以下の説明に従って事前に地図ファイルをインストールしてください．  
  - [https://github.com/AI-Robot-Book-Humble/chapter4/tree/main/map](https://github.com/AI-Robot-Book-Humble/chapter4/tree/main/map) 
