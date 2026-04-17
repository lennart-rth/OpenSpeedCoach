with import <nixpkgs> {}; let
  pythonPackages = python312Packages; # Change to Python 3.12
in
  pkgs.mkShell rec {
    name = "impurePythonEnv";
    venvDir = "./.venv";
    buildInputs = [
      pkgs.stdenv.cc.cc.lib

      git-crypt
      # stdenv.cc.cc # jupyter lab needs

      # for running jupyter lab
      pythonPackages.jupyterlab
      pythonPackages.notebook

      # pythonPackages.python
      pythonPackages.venvShellHook
      pythonPackages.pip

      pythonPackages.numpy
      pythonPackages.pandas
      pythonPackages.matplotlib

      # sometimes you might need something additional like the following - you will get some useful error if it is looking for a binary in the environment.
      taglib
      openssl
      git
      libxml2
      libxslt
      libzip
      zlib
    ];

    # Set the environment variable to the lib directory of the C compiler
    # to get eg. matplotlib to work
    LD_LIBRARY_PATH = "${pkgs.stdenv.cc.cc.lib}/lib";

    # Run this command, only after creating the virtual environment
    postVenvCreation = ''
      unset SOURCE_DATE_EPOCH

      # pip install -r requirements.txt
    '';
    # for auto launching the server inlcude it up there (but there is a bug)
    # python -m ipykernel install --user --name=myenv4 --display-name="myenv4"

    shellHook = ''
      # define stuff that should be done when entering the shell
    '';

    # Now we can execute any commands within the virtual environment.
    # This is optional and can be left out to run pip manually.
    postShellHook = ''
      # allow pip to install wheels
      unset SOURCE_DATE_EPOCH
    '';

    # run the jupyter lab in the shell with `jupyter lab`
    # then get the url wiht the token and paste it in vscode under 'select another kernek/choose exisiting kernel'
  }
