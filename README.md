# DonkeyCar_for_PIUS
---
### 概要
JetsonNanoで使用できるDonkeyCarの最新バージョンであるDonkeyCar-4.5.1でPIUSを扱うためのパッチとスクリプトです。

### 使用方法
`parts`, `pipeline` でDonkeyCar公式のリポジトリ内の `donkeycar` のサブディレクトリの同名ディレクトリ・ファイルを上書きしてください。
`manage.py` はPIUSで使用する実行ディレクトリの同名ファイルを置き換えて使用してください。
スクリプトの使用方法は、引数なしで実行することで表示されます。

### 内容
`parts`, `pipeline` には、RCカー用に設計された運転・学習・推論用のシステムをPIUSで使用するために必要な変更を加えたファイルが格納されています。
`manage.py` は、変更を加えたファイルを使用して実際にDonkeyCarによるPIUSの制御を行うためのスクリプトです。
