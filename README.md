

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

- ドライバー
  - [x] CANドライバの作成
  - [x] LANドライバの作成
- システム
  - [x] SystemManagerの作成
  - [x] State, Cmd, Stackの初期化
  - [x] ControlManagerからState, Cmd, Stackのメモリ共有
  - [x] LANドライバを食べる
  - [x] 空の状態でRUN
  - [ ] システム内GUI(display module)
    - [x] SMに対応した顔表示を作成
    - [ ] チラつき防止
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
    - [ ] Cybergearドライバを作成
  - [ ] コマンドリストに対応した実行を作成
  - [ ] ベース制御技術
  - [ ] コア制御技術
- 外部システム連携
  - [x] 共通のnode_state, node_cmd, node_state_machineを使用
  - [x] node_defに固有定数をまとめる
  - [x] m5mf用のコマンドリストを作成
- [ ] 外部システム
  - [ ] RobotAPIノード(B Node)の作成
    - [ ] 単軸ノードからの移植
    - [ ] UDPによる通信確認
  - [ ] Robot操作ノード(B Node)の作成
    - [ ] Bluetoothゲームパッドノードからの移植
  - [ ] ノード間連携
  - [ ] GUI設計
    - [ ] UDP通信スキーマ作成
    - [ ] 画面実装


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
