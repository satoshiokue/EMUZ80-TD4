# EMUZ80-TD4
TD4 Mezzanine board for EMUZ80

クロック生成と命令デコードをEMUZ80のPICで行うことで拡張性を高めたTD4 4bitCPUシステムです。  
![MEZTD4ROM](https://github.com/satoshiokue/EMUZ80-TD4/blob/main/emuTD4_ROM.jpeg)  
16bytes ROMボードを接続したシステムの全幅は338mm  

![MEZTD4](https://github.com/satoshiokue/EMUZ80-TD4/blob/main/emuTD4.jpeg)  
ROMボードを分離するとPICがプログラムデータをデータバスに出力します。  

スライドスイッチでクロックの[Auto/Manual]を切り替え、タクトスイッチでManualクロックを入力します。  
スイッチをManualからAutoに切り替え、タクトスイッチを押すとクロックが再開されます。  

ソースコードは電脳伝説さんのEMUZ80用main.cを元に改変してGPLライセンスに基づいて公開するものです。  

## MEZTD4  
https://github.com/satoshiokue/MEZTD4

## ファームウェア
emuTD4.cをEMUZ80で配布されているフォルダemuz80.X下のmain.cと置き換えて使用してください。  
ターゲットのPICを適切に変更してビルドしてください。  

## PICプログラムの書き込み
EMUZ80技術資料8ページにしたがってPICに適合するemuTD4_Qxx.hexファイルを書き込んでください。  

またはArduino UNOを用いてPICを書き込みます。  
https://github.com/satoshiokue/Arduino-PIC-Programmer

PIC18F47Q43 - emuTD4_Q43.hex  
PIC18F47Q83 - emuTD4_Q8x.hex  
PIC18F47Q84 - emuTD4_Q8x.hex  

## TD4プログラムの改編
ファームウェアの配列rom_data[]に格納するとTD4で実行できます。

## 参考）EMUZ80
EUMZ80はZ80CPUとPIC18F47Q43のDIP40ピンIC2つで構成されるシンプルなコンピュータです。

![EMUZ80](https://github.com/satoshiokue/EMUZ80-6502/blob/main/imgs/IMG_Z80.jpeg)

電脳伝説 - EMUZ80が完成  
https://vintagechips.wordpress.com/2022/03/05/emuz80_reference  

EMUZ80専用プリント基板 - オレンジピコショップ  
https://store.shopping.yahoo.co.jp/orangepicoshop/pico-a-051.html
