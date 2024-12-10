

以下を引用・参考にした。

(引用)PlatformIO IDE 向け M5Stack 定型コード環境
https://github.com/3110/m5stack-platformio-boilerplate-code





`main_task`, `ctrl_task`, `comm_receive_task`の3つのタスクスレッドが働いており、その中で`SystemManager`クラスの`sys_manager`と
`ControlManager`クラスの`ctrl_manager`が仕事をしている。

`SystemManager`は汎用的なシステム管理クラスであり、外部との連携と内部システム管理を担当する。`ControlManager`はユニークな機能クラスであり、ノード固有の機能を担当する。

### 通信デバイスの取り扱い

CANユニットやLANユニットは、`main.cpp`の中の`setup`関数でPIN定義・初期化を行い、各タスクスレッド内の初期化パートで`SystemManager`クラスの`sys_manager`もしくは
`ControlManager`クラスの`ctrl_manager`の`init`関数の引数ポインタを使ってマネージャーが持つモジュールに登録する。

### 略語

SM: State-Machine
MF: Mechanical-Frame

## ノードの機能追加のために編集する部分

### コマンドの追加・編集

`m5mf_cmd_list.hpp`にある`M5MF_CMD_LIST`を自由に編集して、外部あるいは内部から命令可能なコマンドを作成する。
コマンドに付随して送るコマンドデータがあれば、同じ`Common`フォルダ内部に追加する。

ここで、`0 - 99`はSystem向けのコマンドであり、`100 - `はControl向けのコマンドである。

### ノードの状態量および状態リストの追加・編集

`Control`フォルダの中に`st_control_state.hpp`を置いて編集する。リストも同様に`Control`フォルダの中に置く。

# 実装整理

- ISSUE
  - [ ] Logging機能がしんでる
    - 症状：Start_Loggingコマンドを受け取ると、M5Stackが再起動する
  - [ ] 立ち上げ順序が変わると機能しない
    - 症状：M5Stackを再起動させるとPC側のノードも再起動する必要がある

- ドライバー
  - [x] CANドライバの作成
  - [x] LANドライバの作成
- システム
  - [x] SystemManagerの作成
  - [x] State, Cmd, Stackの初期化
  - [x] ControlManagerからState, Cmd, Stackのメモリ共有
  - [x] LANドライバを食べる
  - [x] 空の状態でRUN
  - [x] システム内GUI(display module)
    - [x] SMに対応した顔表示を作成
    - [x] チラつき防止
  - [x] 非常停止外部物理ボタン作成
  - [x] コマンドスタックのExecutor作成
  - [x] コマンドリストに対応した実行を作成
    - [x] State Machine
    - [x] Data Stream and logging
    - [x] Alert and Error
  - [x] Heartbeat Moduleの作成
- コントロール
  - [x] ControlManagerの作成
  - [x] State, Cmd, Stackの初期化
  - [x] CANドライバを食べる
  - [x] 空の状態でRUN
  - [ ] Servoドライバを作成
    - [ ] M5RollerCANドライバを作成
    - [x] Cybergearドライバを作成
  - [ ] コマンドリストに対応した実行を作成
  - [ ] ベース制御技術
  - [ ] コア制御技術
- 外部システム連携
  - [x] 共通のnode_state, node_cmd, node_state_machineを使用
  - [x] m5mf_defに固有定数をまとめる
  - [x] m5mf用のコマンドリストを作成
- [ ] 外部システム
  - [ ] RobotAPIノード(B Node)の作成
    - [x] 単軸ノードからの移植
    - [x] UDPによる通信確認
    - [x] Ready -> Stable, Repair -> Stableに移行するようにprocess書いた
    - [x] 自動的にデバイス(M5MF)とUDP通信を試みるようにprocess書いた
    - [x] Robot操作ノードと連携(子ノードとして呼び出し)
    - [ ] Warning処理と復旧処理
    - [ ] Error処理と復旧処理
  - [ ] Robot操作ノード(B Node)の作成
    - [x] Bluetoothゲームパッドノードからの移植(Windows専用ノードになった)
    - [ ] (操作コマンドが決まったら)パッドボタンを割り当てて操作コマンド生成
      - [x] Logging
      - [x] State Stream
  - [x] ノード間連携
    - [x] WindowsJoycon -> M5MF API
  - [ ] GUI設計
    - [ ] UDP通信スキーマ作成
    - [ ] 画面実装
  - [ ] ゲームパッド
    - [x] 1ボタン -> Servo ON/OFF
    - [x] 2ボタン -> Force Stop / Release Force Stop
    - [ ] 4ボタン + 上/下 -> Speed Up / Down
    - [x] 7ボタン -> (MF)Control mode 変更
    - [x] 8ボタン -> Mechanical Frame 変更
    - [ ] 5ボタン(左Shift) + 4ボタン + 上/下 -> (非MF)Control Modeの硬さ Up / Down
    - [ ] 5ボタン(左Shift) + 7ボタン -> (非MF)Control Mode 変更
    - [x] 11ボタン -> StateStream START/STOP
    - [x] 12ボタン -> Logging START/STOP
    - [ ] Axisボタン -> ロボット移動

# TODO
- 2024.12.3
  - [x] CANドライバーの作成
  - [x] LANドライバーの作成
  - [x] sys_managerにLANドライバーを食べさせる
  - [x] UDP送信部の作成
  - [x] UDP受信部の作成
  - [x] ControlManagerのデータをSystemManagerに渡す
- 2024.12.4
  - [x] Heartbeat Moduleの作成
  - [x] コマンドスタックのExecutor作成
  - [x] システム内GUI(display module)
    - [x] SMに対応した顔表示を作成
  - [x] 非常停止外部物理ボタン作成
- 2024.12.5
  - [x] Windows版ゲームパッドノード(B Node)
- 2024.12.9
  - [ ] Mechanical Frame実装
    - [ ] MF:0 NOBODY
    - [ ] MF:1 ALLJOINT
    - [ ] MF:2 SCARA