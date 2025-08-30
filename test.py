import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--debug", action="store_true", help="Enable debug mode")
args = parser.parse_args()

# Use the command-line argument to set DEBUG
DEBUG = True if args.debug else False

print("DEBUG MODE" if DEBUG else "PRODUCTION")