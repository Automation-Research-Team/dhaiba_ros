$ mkdir build
$ cd build/
$ cmake ..
$ make

--------------------------------------------------------------------------------
Shapebox を表示

$ ./shapebox1

Mac 側の DhaibaWorks で GetOnlineTopicList をした後、
TrialShapebox1 のプロパティで UseTransparency を No から Yes に変更する。

--------------------------------------------------------------------------------
Mesh(STL) を表示

$ ./stl1 ../link_0.stl
$ ./stl3 ../link_0.stl

Mac 側の
  Dhaiba Suite v2.18.22080 for macOS/DhaibaWorks V2/Packages/DhaibaConnect
ディレクトリ配下に Data というディレクトリを作成しておく。
画像が出るまで時間が掛かる時がある。

"stl1/TrialSTL1.Mesh::Definition_BinaryFile" トピックを subscribe
$ ./subscriber

"stl3/TrialSTL3.Mesh::Definition_BinaryFile" トピックを subscribe
$ ./subscriber 3

--------------------------------------------------------------------------------
Shapebox と Mesh を２つずつ表示

$ ./multiple

トピックの作成順を Shapebox Mesh とする
$ ./multiple bm

トピックの作成順を Mesh Shapebox とする
$ ./multiple mb

../link_1.stl と ../link_2.stl を使っているので、先に配置しておくこと。

Mesh の準備が出来るまでの間、Shapebox は default 状態で表示されるように思う。
Shapebox の default 状態とは scale が 1000x1000x1000 で、グレーの箱。

--------------------------------------------------------------------------------

