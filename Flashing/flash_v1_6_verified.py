#!/usr/bin/env python3
"""Install v1.6 on a C30D with a verified v1.5 sector 0."""
from verified_upgrade import install


if __name__ == "__main__":
    install(
        "v1.6",
        "bfb4f82ebb91cad3d136e79bf177fecb22d27360d43f1d278a4f4f514a823874",
        "87fead14af4644975f4602afb0ca080f2943ae92aa9d83c4a4e6d706cb73b4ec",
    )
