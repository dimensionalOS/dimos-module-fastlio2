{
  description = "FAST-LIO (non-ROS) dev environment";

  inputs = {
    # Pin to nixos-24.11 (GCC 13) — nixos-unstable has GCC 15 which has
    # a brace-init incompatibility with glibc's pthread headers.
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
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
          ] ++ pkgs.lib.optionals pkgs.stdenv.isLinux [
            libnl  # needed by libpcap (PCL→VTK transitive on Linux only)
          ];

          shellHook = ''
            echo "FAST-LIO-NON-ROS dev shell ready"
            echo "Build: mkdir -p build && cd build && cmake .. && make"
          '';
        };
      });
}
