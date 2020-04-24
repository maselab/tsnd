# tsnd
Library to handle TSND series, IMU sensors produced by ATR Japan.

# Reqirement
 pip install pyserial

# How to clone this repository
For git clone, please use the following process
```Bash
git clone https://github.com/maselab/tsnd.git
cd tsnd
git submodule update -i
echo "common_utils.py" > .git/modules/tsnd/utils/info/sparse-checkout
echo "thread_utils.py" >> .git/modules/tsnd/utils/info/sparse-checkout
(cd tsnd/utils; git config core.sparsecheckout true; git read-tree -mu HEAD)
# git submodule update --remote # if necessary
```

# NOTE
There are many lacks of functions that is not necessary on related project.
We are happy if you contribute to add such lacking functions. Thanks.

