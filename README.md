# K4aGrabber (Masutani Lab version)
升谷 保博  
大阪電気通信大学  
2020年4月15日

[オリジナルの`README.md`はこちら](README-original.md)

## 概要

- Haipeng Wang氏が公開しているAzure Kinect DKのPCL用Grabber[K4aGrabber](https://github.com/forestsen/K4aGrabber)を簡単化したものです．
- オリジナルとの違い．
  - Kinectを2台使うプログラムを削除．Open3Dに依存しないようにした．
  - PCL 1.10に対応させた．
  - CMakeの際にAzure Kinect SDKのバージョンに依存しないようにした．
  - 杉浦氏の[Findk4a.cmake](https://gist.github.com/UnaNancyOwen/90b898366eb908d29cb4c2b509ab6cfa#file-findk4a-cmake)を導入した．
  - ディレクトリ構成を変更．使わないファイルを削除．

## 開発環境
以下の環境で開発・動作確認しています．
- Windows 10 64bit版
- Visual Studio 2019 x64
- Azure Kinect SDK v1.4.0
- Point Cloud Library 1.10.1 AllinOne (VS2019 64bit用)

## 準備
- [GitHubのpclのRelease](https://github.com/PointCloudLibrary/pcl/releases)の中のWindows用AllInOne `PCL-X.X.X-AllInOne-msvcXXXX-winXX.exe`をダウンロードし実行．
- [Azure Kinect Sensor SDK download](https://docs.microsoft.com/ja-jp/azure/Kinect-dk/sensor-sdk-download)からWindows用インストーラ`Azure Kinect SDK X.X.X.exe` をダウンロードし実行．
- 環境変数 
  - `K4A_DIR=C:\Program Files\Azure Kinect SDK v1.4.0` を追加
  - `Path`の値の並びに`%K4A_DIR%\sdk\windows-desktop\amd64\release\bin`を追加．

## ビルド
- [K4aGrabber](https://github.com/MasutaniLab/K4aGrabber)をクローン．
- CMake
- Visual Studio でReleaseでビルド．

## 使い方
- Azure Kinect DKをUSB 3.0のポートに接続する．
- `build\Release\viewer.exe` を実行．
- PCLのVisualizerが表示される．

## 既知の問題・TODO
- 検証不十分．

