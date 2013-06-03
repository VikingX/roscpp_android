cmd_exists() {
    command -v $1 > /dev/null 2>&1
}

die() {
    echo $1
    exit 1
}

download() {
    cmd_exists curl && curl -L $1 -O || wget $1
}

download_bz2() {
    echo "downloading $1"
    ( cmd_exists curl && curl -L $1 || wget -O - $1 ) | tar jx -C $2
}

download_gz() {
    echo "downloading $1"
    ( cmd_exists curl && curl -L $1 || wget -O - $1 ) | tar zx -C $2
}

download_zip() {
    cmd_exists unzip || die 'could not find unzip'

    echo "downloading $1"

    tmpdir=$(mktemp -d /tmp/rba.XXXX)
    tmpfile=$tmpdir/gtest.zip
    ( cmd_exists curl && curl -L $1 -o $tmpfile || wget $1 -O $tmpfile ) && unzip $tmpfile -d $2
    rm -rf $tmpdir
}
