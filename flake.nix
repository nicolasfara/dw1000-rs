{
  description = "Nix development environment for dw1000-rs";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };

        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" ];
          targets = [ "thumbv7em-none-eabihf" "thumbv7em-none-eabi" ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            rustToolchain
            probe-rs-tools
            cargo-binutils
            pkg-config
            libiconv
          ] ++ lib.optionals stdenv.isDarwin [
            darwin.apple_sdk.frameworks.Security
            darwin.apple_sdk.frameworks.CoreFoundation
            darwin.apple_sdk.frameworks.SystemConfiguration
          ];

          RUST_SRC_PATH = "${rustToolchain}/lib/rustlib/src/rust/library";

          shellHook = ''
            echo "=========================================================="
            echo "⚡ dw1000-rs Nix Development Shell Loaded ⚡"
            echo "Rust version:  $(rustc --version)"
            echo "rust-analyzer: $(which rust-analyzer)"
            echo "probe-rs:      $(which probe-rs)"
            echo "RUST_SRC_PATH: $RUST_SRC_PATH"
            echo "=========================================================="
          '';
        };
      }
    );
}
