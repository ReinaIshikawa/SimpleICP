# ICP
simple ICP using PCL to understand the algorithm

opencvとPCLをインストール・ビルド等を実行の上，
CMAKEを実行
ICPalgorithm/build内のReleaseフォルダまたはDeugフォルダ内にできるexeファイルを実行すれば，結果が見れる

## コード内容についての注釈
1. 簡単化するため，適当な点群を発生させる(直方体)　これを"original cloud"とする
2. 点群を複製し，適当に平行移動回転させる　これを"current cloud" とする．　
      - この点群を，original点群に向けて移動を最適化によって推定し，移動・回転させて，オリジナルの点群にfusionしていく
3. 今回用いたのは point-to-planeである
      - 最近傍の点を用いて，各点の法線を求めた
4. 最適化については，回転角が小さいと仮定の上(ここでは10度回転. 30度くらいまではOK), 近似・線形変換を行う．
      - 詳しくは
      Kok-Lim Low, Linear Least-Squares Optimization forPoint-to-Plane ICP Surface Registration
      の論文を参照
      https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
5. 出力結果は
      白点群： original cloud
      青点群: current cloud
      赤点群: transformed cloud
      
      
