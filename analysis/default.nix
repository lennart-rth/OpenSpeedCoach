{ pkgs ? import <nixpkgs> {} }:

let
  # This creates a Python version that has all your libraries pre-baked into it
  myPython = pkgs.python312.withPackages (ps: with ps; [
    scipy
    numpy
    pandas
    matplotlib
    scikit-learn
    jupyterlab
    notebook
    pip
    plotly
  ]);
in
pkgs.mkShell {
  name = "science-env";

  buildInputs = [
    myPython
    pkgs.git-crypt
    pkgs.taglib
    pkgs.openssl
    pkgs.git
    pkgs.libxml2
    pkgs.libxslt
    pkgs.libzip
    pkgs.zlib
  ];

  # This helps SciPy and Matplotlib find necessary C libraries
  LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
    pkgs.stdenv.cc.cc.lib
    pkgs.zlib
  ];

  shellHook = ''
    echo "Nix Python environment loaded!"
    echo "Python version: $(python --version)"
    echo "SciPy location: $(python -c 'import scipy; print(scipy.__file__)' 2>/dev/null || echo 'Not found')"
  '';
}