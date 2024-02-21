import serial
import cv2

import numpy as np
import matplotlib.pyplot as plt
import math
import cmath
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import os
import sys

# 接続するT14REのキャプチャIDの設定
CaptureDeviceIndex = 0
# 接続するT14REのMMICコマンドインターフェースの設定
MmicCommandPort = 'COM3'
# コンフィグファイルの設定
CfgFile = 'T14RE_2D_Short.cfg'

# 取り込むデータサイズの指定
Sample = 256
Tx = 3
Rx = 4
Chirpset = 16
Frame = 1



# MMICにコマンドを送信する
def SendCommand(serialCommand, command):
    # 改行文字をLFに変更して送信
    command = command.replace('\r', '')
    command = command.replace('\n', '')
    command += '\n'
    serialCommand.write(command.encode())
    
    # 応答待ち
    ret = serialCommand.read_until(b'\nmmwDemo:/>')
    print(ret)

    # 応答確認
    # 空白、コメント、sensorResetはDoneを返さない
    if (
        (command.strip() != '') and
        (command.strip()[0] != '%') and
        (command != 'sensorReset\n')
    ):
        rets = ret.decode().split('\n')
        if (rets[len(rets) - 2] != 'Done'):
            return False

    return True



# コンフィグファイルをMMICに送信する
def SendCfg(filename):
    # Cfgファイルの内容を取得する
    file = open(filename)
    commands = file.readlines()
    file.close()

    # MMICコマンド用のシリアルポートを開く
    serialCommand = serial.Serial(MmicCommandPort, 115200)
    serialCommand.reset_output_buffer()
    serialCommand.reset_input_buffer()

    #コマンドの送信
    for command in commands:
        if SendCommand(serialCommand, command) == False:
            print("Error")
            serialCommand.close()
            return False
    
    serialCommand.close()
    return True


def CACFAR(data, guard : int, windowsize : int, offset) -> list:
    cfar_data = [None]*len(data)
    cfar_result = []
    checkIndex = []
    prev = data[0]

    #デシベルに変換
    for i in range(len(data)):
        cfar_data[i] = 20*math.log10(data[i])

    #山の頂点の位置を求める
    if len(cfar_data) > 2:
        for i in range(1, len(cfar_data)):
            if cfar_data[i] - cfar_data[i - 1] <= 0 and prev > 0:
                checkIndex.append(i-1)
            prev = cfar_data[i] - cfar_data[i - 1]
        if cfar_data[-1] > cfar_data[0]:
            checkIndex.append(len(cfar_data)-1)

    #CFARを行う
    for index in checkIndex:
        th = 0
        th_count = 0
        for i in range(windowsize):
            if index + (guard+1) + i >= len(cfar_data):
                pass
            else:
                th += cfar_data[index + (guard+1) + i]
                th_count += 1
            
            if index - (guard+1) - i < 0:
                pass
            else:
                th += cfar_data[index - (guard+1) - i]
                th_count += 1
        
        #平均計算
        th = th/th_count

        if th+offset < cfar_data[index]:
            cfar_result.append(index)
    return cfar_result

def fft(rawdata, Sample, convertToAbs=False) -> list:
    array = np.zeros(int(len(rawdata)/2),dtype=np.complex128)
    array = np.reshape(array, [int(len(array)/Sample), Sample])
    rows, columns = array.shape

    for i in range(rows):
        for j in range(columns):
            array[i][j] = complex(rawdata[i*Sample + 2*j], rawdata[i*Sample + 2*j+1])
        array[i] = np.fft.fft(array[i])
        pass
        
    if convertToAbs == True:
        abs_array = np.zeros(int(len(rawdata)/2))
        abs_array = np.reshape(abs_array, [int(len(abs_array)/Sample), Sample])
        for i in range(rows):
            for j in range(columns):
                abs_array[i][j] = abs(array[i][j])

        array = abs_array

    return array

def fftnumpy(NumpyRawData : np.ndarray, Sample=256, convertToAbs=False):

    valI = NumpyRawData.T[0]
    valQ = NumpyRawData.T[1] * 1j
    IQ_val = valI+valQ
    IQ_val = np.reshape(IQ_val, [int(IQ_val.size/Sample), Sample])
    array = np.fft.fft(IQ_val)
        
    if convertToAbs == True:
        array = np.abs(array)

    return array

def distance(fftLength, peak : list) -> list:
    distResult = []
    for i in peak:
        distResult.append(((i / fftLength) * 2.29 * 300) / (2 * 30))
    return distResult

def velocity(fftdata, peak : list):
    rx = 4
    tx = 3
    c = 299792458 #光速度 [m/s]
    T_c = 660 * math.pow(10, -6) #チャープ間隔 [s]
    lam = c / (79*math.pow(10,9)) #波長

    v_data = [None]*len(peak)
    for i in range(len(peak)):
        phi = cmath.phase(fftdata[rx*tx][peak[i]]) - cmath.phase(fftdata[0][peak[i]])
        if phi > math.pi:
            phi = -(2*math.pi - phi)
        elif phi < -math.pi:
            phi = 2*math.pi + phi

        v_data[i] = (phi * lam) / (4 * math.pi * T_c)
    return v_data

def angle(fftdata, peak : list):
    rx = 4
    tx = 3
    c = 299792458 #光速度 [m/s]
    T_c = 660 * math.pow(10, -6) #チャープ間隔 [s]
    lam = c / (79*math.pow(10,9)) #波長

    arg_data = [None]*len(peak)
    for i in range(len(peak)):
        phi = cmath.phase(fftdata[1][peak[i]]) - cmath.phase(fftdata[0][peak[i]])
        if phi > math.pi:
            phi = -(2*math.pi - phi)
        elif phi < -math.pi:
            phi = 2*math.pi + phi
        arg_data[i] = math.asin(phi/math.pi)
    return arg_data

class Data():
    def __init__(self) -> None:
        pass
    def SetDataInfo(self, CaptureDeviceIndex, Sample, Tx, Rx, Chirpset, Frame) -> None:
        self.CaptureDeviceIndex = CaptureDeviceIndex
        self.Sample = Sample
        self.Tx = Tx
        self.Rx = Rx
        self.Chirpset = Chirpset
        self.Frame = Frame

    def DataStart(self):
        # VideoCaptureオブジェクトを取得する
        # Window: Microsoft Media Foundationを使用する設定
        self.capture = cv2.VideoCapture(self.CaptureDeviceIndex, cv2.CAP_MSMF)
        self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.capture.set(cv2.CAP_PROP_FORMAT, -1)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.Sample * 2 * self.Rx)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.Tx * self.Chirpset * self.Frame)

        #Rawデータ保存用のリストを作成
        self.__RawData = [None]*(2*self.Sample*self.Tx*self.Rx*self.Chirpset*self.Frame)
    def DataStop(self):
        self.capture.release()
        pass


    def GetRawData(self) -> list:
        ret, data = self.capture.read()

        # デコード
        # MSMFでは1次元データフォーマットで取得できる
        if (ret):
            index = 0
            rawDataIndex = 0
            for fr in range(self.Frame*self.Chirpset*self.Tx*self.Rx):
                for ss in range(self.Sample):
                    valueI = data[0][index + 0] + data[0][index + 1] * 256
                    if (valueI >= 0x8000):
                        valueI -= 0x10000
                                
                    valueQ = data[0][index + 2] + data[0][index + 3] * 256
                    if (valueQ >= 0x8000):
                        valueQ -= 0x10000
                    self.__RawData[rawDataIndex] = valueI
                    self.__RawData[rawDataIndex+1] = valueQ
                    rawDataIndex += 2
                    index += 4
        return self.__RawData
    

    def GetRawDataFast(self, sliceSize=-1) -> np.ndarray:
        ret, data = self.capture.read()
        #どこまでのデータをデコード取得するか
        if sliceSize > 0 and sliceSize%4 == 0:
            data = data[0][:sliceSize]
            
        if(ret):
            #符号なし整数型から符号あり整数型に変換
            data = data.astype("int")
            
            #I下位ビット, I上位ビット, Q下位ビット, Q上位ビットに分割
            data = data.reshape([int(data.size/4), 4])
            I_LowByte = data.T[0]
            I_HighOrderByte = data.T[1] * 256
            Q_LowByte = data.T[2]
            Q_HighOrderByte = data.T[3] * 256

            # IQ信号をデコード
            valueI = (I_LowByte + I_HighOrderByte)
            valueI = np.where(valueI >= 0x8000, valueI-0x10000, valueI)
            valueQ = Q_LowByte + Q_HighOrderByte
            valueQ = np.where(valueQ >= 0x8000, valueQ-0x10000, valueQ)

            #二つの配列を結合
            rawdata = np.vstack([valueI,valueQ]).T
        return rawdata
        
    def __del__(self):
        #デストラクター
        pass


class GUI(tk.Frame):
    def __init__(self, master = None):

        #親クラスのコンストラクタ呼び出し, ウィンドウサイズ, タイトル設定
        super().__init__(master)
        self.master = master
        master.geometry("900x700")
        master.title("T14re Demo")

        #マルチスレッド設定
        self.thread1 = threading.Thread(target=self.fft_graph_plot) #FFTグラフ表示
        self.thread2 = threading.Thread(target=self.scatter_graph_plot) #速度、角度グラフ表示

        #グラフ設定
        self.fig = plt.Figure(figsize=(4.5,4.5), dpi=100) #グラフのサイズ設定
        self.fig2 = plt.Figure(figsize=(4.5,4.5), dpi=100)
        self.ax = self.fig.add_subplot(111) #FFT表示用の軸
        self.ax2 = self.fig2.add_subplot(211) #速度表示用の軸
        self.ax3 = self.fig2.add_subplot(212) #角度表示用の軸
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)  #FFT表示用のキャンバス
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=root)  #それ以外表示用のキャンバス
        self.canvas.get_tk_widget().grid(row=0, column=0, rowspan=4, columnspan=4, sticky='n') #グラフの配置設定
        self.canvas2.get_tk_widget().grid(row=0, column=4, rowspan=4, columnspan=4, sticky='n')

        #角度グラフに描画する扇型作成
        for i in range(2, 10, 2):
            x = np.arange(-i*math.cos(math.pi/4), i*math.cos(math.pi/4), 0.02)
            y = [math.sqrt(i**2 - j**2) for j in x]
            self.ax3.plot(x, y, color='black', linewidth=1, alpha=0.5)
        x = np.arange(0, 10)
        y = np.arange(0, 10)
        self.ax3.plot(x, y, color='black', linewidth=1, alpha=0.5)
        x = np.arange(0, -10, -1)
        y = np.arange(0, 10)
        self.ax3.plot(x, y, color='black', linewidth=1, alpha=0.5)
        #グラフクリア用変数の初期化
        self.FftLine, = self.ax.plot(0,0,alpha=0)
        self.FftScatter = self.ax.scatter(0,0,alpha=0)
        self.VelScatter = self.ax2.scatter(0,0,alpha=0)
        self.ArgScatter = self.ax3.scatter(0,0,alpha=0)
        
        #リストボックス設定
        self.listbox = tk.Listbox(self.master, width=50)
        self.listbox.grid(row=5, column=0, padx=10, pady=10)

        #ボタン設定
        self.button1 = tk.Button(self.master, text="スタート", command=self.start_button_click)
        self.button1.place(x=350, y=500)
        self.button2 = tk.Button(self.master, text="ストップ", command=self.stop_button_click)
        self.button2.place(x=350, y=530)
        self.button3 = tk.Button(self.master, text="Config", command=self.config_button_click)
        self.button3.place(x=480, y=455)
        self.button4 = tk.Button(self.master, text="設定", command=self.cfar_setting_click)
        self.button4.place(x=600, y=550)

        #テキストボックス
        self.txtbox1 = tk.Entry(self.master, width=20)
        self.txtbox1.place(x=350, y=460)

        #Cfar設定
        self.gard = 8
        self.window = 16
        self.th = 20

        #ラベル
        self.label1 = tk.Label(self.master, text="CFAR設定", font=("MSゴシック", "15"))
        self.label1.place(x=600, y=455)
        self.label2 = tk.Label(self.master, text=f"ガード      {self.gard}")
        self.label2.place(x=600, y=485)
        self.label3 = tk.Label(self.master, text=f"ウィンドウ {self.window}")
        self.label3.place(x=600, y=505)
        self.label4 = tk.Label(self.master, text=f"閾値       {self.th}")
        self.label4.place(x=600, y=525)
        self.label5 = tk.Label(self.master, text=f"現在設定中のCOMポート : {MmicCommandPort}", font=("MSゴシック", "10"))
        self.label5.place(x=350, y=580)
        self.label6 = tk.Label(self.master, text=f"現在設定中のキャプチャID : {CaptureDeviceIndex}", font=("MSゴシック", "10"))
        self.label6.place(x=350, y=610)


        #制御用メンバ変数
        self.is_running = False
        self.CfgFile = None #Cfgのファイルパスが入る
        self.LOOP_func_is_run = False
        
    def start_button_click(self):
        #コンフィグファイル送信
        if self.txtbox1.get() == "":
            tk.messagebox.showinfo("エラー", "Cfgファイルを設定してください")

        elif (SendCfg(self.CfgFile) == False):
            tk.messagebox.showinfo("エラー", "Cfgファイルの送信に失敗しました")

        else:
            #Dataクラスのインスタンス作成
            self.MyDataClass = Data()
            self.MyDataClass.SetDataInfo(CaptureDeviceIndex, Sample, Tx, Rx, Chirpset, Frame)
            self.MyDataClass.DataStart()

            #起動中かのフラグをTrueに
            self.is_running = True
            #スタートボタンを無効化する
            self.button1["state"] = "disable"
            #LOOP()関数を呼び出し
            self.LOOP()

    def stop_button_click(self):
        #スタートボタンを有効化
        self.button1["state"] = "normal"
        #起動中かのフラグをFalseに
        self.is_running = False
        pass

    def config_button_click(self):
        #ファイルダイアログを開く
        fTyp = [("コンフィグファイル", "*.cfg")]
        iDir = os.path.abspath(os.path.dirname(__file__))
        self.CfgFile = tk.filedialog.askopenfilename(filetypes=fTyp, initialdir=iDir)
        #テキストボックスにファイル名を追加
        self.txtbox1.delete(0, tk.END)
        self.txtbox1.insert(tk.END,self.CfgFile.split("/")[-1])
        pass
    def cfar_setting_click(self):
        self.gard = tk.simpledialog.askinteger("ガード", "ガードサイズを入力してください", minvalue=0, maxvalue=100)
        self.window= tk.simpledialog.askinteger("ウィンドウ", "ウィンドウサイズを入力してください", minvalue=0, maxvalue=100)
        self.th = tk.simpledialog.askinteger("閾値", "閾値を入力してください", minvalue=0, maxvalue=100)

        self.label2["text"] = f"ガード      {self.gard}"
        self.label3["text"] = f"ウィンドウ {self.window}"
        self.label4["text"] = f"閾値       {self.th}"
        pass

    def LOOP(self):
        if self.is_running:
            self.LOOP_func_is_run = True
            #FFTデータ取得
            raw = self.MyDataClass.GetRawDataFast()
            self.fft_result = fftnumpy(raw, Sample)
            #FFTの絶対値データを作成
            self.fftAbs = np.abs(self.fft_result)

            #距離計算
            self.cfar = CACFAR(self.fftAbs[0], self.gard, self.window, self.th)
            self.distList= distance(len(self.fftAbs[0]), self.cfar)
            #速度計算
            self.velList = velocity(self.fft_result, self.cfar)
            #角度計算
            self.argList = angle(self.fft_result, self.cfar)

            #マルチスレッド起動
            if self.thread1.is_alive() == False:
                self.thread1 = threading.Thread(target=self.fft_graph_plot) #FFTグラフ表示
                self.thread1.start()
                pass
            if self.thread2.is_alive() == False:
                self.thread2 = threading.Thread(target=self.scatter_graph_plot) #その他のグラフ表示
                self.thread2.start()
                pass

            #リストボックス更新
            self.listbox_plot()

            #50msごとに自分自身を呼び出す
            self.master.after(50, self.LOOP)
        else:
            self.LOOP_func_is_run = False
        pass

    def fft_graph_plot(self):
        #グラフのクリア
        #self.ax.cla()でも可能だが, こちらのほうが早い
        self.FftLine.remove()
        self.FftScatter.remove()

        #グラフの描画
        #fftの描画
        self.FftLine, = self.ax.plot(range(256), self.fftAbs[0], color='blue')

        #CFARの結果を散布図として描画
        self.FftScatter = self.ax.scatter(self.cfar, [self.fftAbs[0][i] for i in self.cfar], color="orange", zorder=2)
        #グラフの範囲を自動調整する
        self.ax.relim()

        #描画
        try:
            self.canvas.draw()
        except Exception:
            pass


    def scatter_graph_plot(self):
        #グラフのクリア
        #self.ax2.cla(), self.ax3.cla()でも可能だが, こちらのほうが早い
        self.VelScatter.remove()
        self.ArgScatter.remove()

        #速度の描画
        self.VelScatter = self.ax2.scatter(self.distList, [i*3.6 for i in self.velList], color="orange") #3.6はm/sからkm/hに直すため

        #角度の描画
        self.ArgScatter = self.ax3.scatter([i*math.sin(j) for i, j in zip(self.distList, self.argList)],
                          [i*math.cos(j) for i, j in zip(self.distList, self.argList)], color="orange") 

        #軸の範囲指定
        self.ax2.set_xlim([0, 10])
        self.ax2.set_ylim([-5,5])
        self.ax3.set_xlim([-10, 10])
        self.ax3.set_ylim([0,10])
        
        #描画
        try:
            self.canvas2.draw()
        except Exception:
            pass

    def listbox_plot(self):
        #リストボックスに表示
        self.listbox.delete(0, "end")
        for i, j, k in zip(self.distList, self.velList, self.argList):
            self.listbox.insert("end", f"{round(i, 2)}[m]  {round(j*3.6, 2)}[Km/h]  {round(k*(180/math.pi), 2)}°")
    
    def click_close(self):
        #ループストップ
        self.is_running = False

        #データ取得ののストップ
        #self.MyDataClassの定義前に実行されるとAttributeErrorが出る。
        try:
            self.MyDataClass.DataStop()
        except AttributeError:
            pass

        #スレッドの終了
        #thread.join()やsleepでスレッド終了を待つと、canvas.draw()がメインスレッドにアクセスする関係上、完全にフリーズする
        #その為、is_alive()で生存確認をし、生きていればmaster.afterでmainloopに戻り、canvas.draw()の処理が終わったかis_alive()で確認する。
        if self.thread1.is_alive() or self.thread2.is_alive() or self.LOOP_func_is_run:
            self.master.after(50, self.click_close)
        else:
            #tkinterの終了
            self.master.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = GUI(master = root)
    root.protocol("WM_DELETE_WINDOW", app.click_close)
    app.mainloop()