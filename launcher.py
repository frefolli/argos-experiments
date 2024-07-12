import os
import itertools

OPTIONS = {
  'INITIAL_CHOICE_STRATEGY': [
    'RANDOM', 'NEAREST'
  ],
  'REVIEW_CHOICE_STRATEGY': [
    'NO_REVIEW',
    'ALWAYS_RANDOM_WHEN_IN_EXCESS',
    'PROBABLE_RANDOM_WHEN_IN_EXCESS'
  ],
  #'COLLISION_AVOIDANCE_POTENTIAL': ['GP', 'LP']
}

def product_dict(**kwargs):
  keys = kwargs.keys()
  for instance in itertools.product(*kwargs.values()):
    yield dict(zip(keys, instance))

def option_key(conf: dict[str, str]) -> str:
  return "__".join(conf.values())

if __name__ == "__main__":
  N = 1
  for conf in product_dict(**OPTIONS):
    destdir = os.path.join("archive/outs", option_key(conf))
    for i in range(1, N + 1):
      os.system("make nox %s" % (" ".join(["%s=%s" % (k,v) for k,v in conf.items()])))
      source_folder = "out/"
      destination_folder = os.path.join(destdir, str(i))
      if not os.path.exists(destination_folder):
        os.makedirs(destination_folder)
      os.rename(source_folder, destination_folder)