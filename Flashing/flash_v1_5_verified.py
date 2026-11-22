#!/usr/bin/env python3
"""Install v1.5 on a C30D with a verified v1.4 sector 0."""
from verified_upgrade import install


if __name__ == "__main__":
    install(
        "v1.5",
        "bd4138c6be3ddfe903535ef7d89869088271cb5fa26d2d37bc5cdfe7031643e1",
        "809b930773b0e7ac8c3c692aecc15ad52ae1f8ae8888d2677e9cf425a9ef2f51",
    )
