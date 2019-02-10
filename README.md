# robot-arm
1. 詳細實作想法、實作細節請見"專題結報.doc"

2. arduino code for the recording of the demostration path and the command of the robot is left at the lab.

3. 操作:

(一)做示範路徑
-上傳arduino code
-在command line輸入: python my_lod_motor.py
操作遙控器以控制手臂夾取目標物件，所存取的示範路徑及動作會被存在data資料夾。

(二)夾取一個任意位置的積木。
-上傳arduino code
-在command line輸入: python averagePath.py ./data1 ./data2 ./data3
其中.py檔後面所輸入的檔案名稱，是示範路徑，我以這種方式將示範路徑傳入代碼，用在徑向坐標上的路徑平均。
