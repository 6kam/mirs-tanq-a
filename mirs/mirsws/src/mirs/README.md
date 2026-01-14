# mirs_mg5
mirs_mg5の標準的機能を備えたROS 2パッケージ

# Attention!
launchはyamlファイルへのパスを自動捜索するように書いているが環境によっては見つからずにエラーが起きる模様。\
cloneした後、それぞれ絶対パスに書き換えることを推奨。

書き換えるのはそれぞれのlaunchファイル内の
```
os.path.join(get_package_share_directory('mirs'),'config','various_file_names.yaml')
```

# installation

```bash
$ cd ~/your_ws/src
$ git clone https://github.com/mirs240x/mirs_mg5.git
$ cd ~/your_ws
$ colcon build --symlink-install
$ source ~/your_ws/install/setup.bash
```
micro-rosのセットアップも必要。
