{
  description = "FAST-LIO (non-ROS) dev environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            cmake
            pkg-config
            eigen
            pcl
            yaml-cpp
            sophus
            llvmPackages.openmp
          ];

          shellHook = ''
            echo "FAST-LIO-NON-ROS dev shell ready"
            echo "Build: mkdir -p build && cd build && cmake .. && make"
          '';
        };
      });
}
