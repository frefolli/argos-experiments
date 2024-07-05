import argparse
import sys

def cli():
  argument_parser = argparse.ArgumentParser()
  return argument_parser.parse_args(sys.argv[1:])

if __name__ == "__main__":
  config = cli()
