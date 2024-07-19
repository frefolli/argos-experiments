import os
import itertools

OPTIONS = {}

def add_executor_stategies():
  OPTIONS.update({
    'COLLISION_AVOIDANCE_POTENTIAL': ['GP', 'LP'],
    'MOTION_APPLIANCE': ['NOISELESS', 'NOISY'],
    'TAKE_OFF_STRATEGY': ['VERTICAL','DIRECT'],
  })

def add_allocator_strategies():
  OPTIONS.update({
    'INITIAL_CHOICE_STRATEGY': ['RANDOM', 'NEAREST'],
    #'REVIEW_CHOICE_STRATEGY': ['NO_REVIEW','ALWAYS_RANDOM_WHEN_IN_EXCESS','PROBABLE_RANDOM_WHEN_IN_EXCESS', 'PROBABLE_MINORITY_WHEN_IN_EXCESS', 'ALWAYS_MINORITY_WHEN_IN_EXCESS'],
    'REVIEW_CHOICE_STRATEGY': ['NO_REVIEW','PROBABLE_RANDOM_WHEN_IN_EXCESS', 'PROBABLE_MINORITY_WHEN_IN_EXCESS'],
  })

#add_executor_stategies()
add_allocator_strategies()

def product_dict(**kwargs):
  keys = kwargs.keys()
  for instance in itertools.product(*kwargs.values()):
    yield dict(zip(keys, instance))

def option_key(conf: dict[str, str]) -> str:
  return "__".join(conf.values())

def stringify_options(options: dict) -> str:
  return (" ".join(["%s=%s" % (k,v) for k,v in options.items()]))

if __name__ == "__main__":
  N = 1
  context = {
    'COLLISION_AVOIDANCE_POTENTIAL': 'LP',
    'MOTION_APPLIANCE': 'NOISELESS',
    'TAKE_OFF_STRATEGY': 'VERTICAL',
    'IDLE_ACTION': 'FINISH'
  }
  for conf in product_dict(**OPTIONS):
    destdir = os.path.join("archive/outs", option_key(conf))
    for i in range(1, N + 1):
      os.system("make nox %s %s" % (stringify_options(conf), stringify_options(context)))
      source_folder = "out/"
      destination_folder = os.path.join(destdir, str(i))
      if not os.path.exists(destination_folder):
        os.makedirs(destination_folder)
      os.rename(source_folder, destination_folder)
