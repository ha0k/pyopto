# PyOpto

Python Interface For Optotune vari-focal Lenses.
You can find the data sheet [here](http://www.optotune.com/downloads2).

## Installation
`pip install git+git+https://github.com/ha0k/pyopto.git`

## Example
```python
from pyopto import Opto

o = Opto("COM4")
o.mode('D') # default mode is current (D)
o.current(100)
o.mode('C') # focal controll mode
o.focul_power(5) # 5 dioptor (focal distance of 20 cm).
o.close()
```