import pytest

from dps5315 import Register

@pytest.mark.parametrize("a,b", [[1,3], [2,4]])
def test_bitwise_and(a, b):
    assert Register(a) & Register(b) == a & b
