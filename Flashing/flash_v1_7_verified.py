#!/usr/bin/env python3
"""Install the archived v1.7 image on the verified v1.6 replacement C30D."""
from verified_upgrade import install


if __name__ == "__main__":
    install(
        "v1.7",
        "e28c611d8ee727043dfc24a17c9a44865c9a799f00c46f4655ecb7b7cdac5ac5",
        "4e9bcafeab040d7f1bd0ad8758948bc180e1c231afa247f8c492fd8a21e4431a",
        archived=True,
    )
