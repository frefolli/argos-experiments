# Progetto di Sistemi Complessi

## Conf

Assert \f$ \Sigma ^ {t \in Targets} t.force < |Spiri|\f$.

## Init
```Rust
let mut distances: Map<Target, Distance>;
lut mut average_distance: Distance = 0.0f;
let mut max_distance: Distance = 0.0f;
let targets: Vector<Target> = GetTargetList();
for target in targets {
  distances[target] = distance(self.position, target.position);
  average_distance += distances[target];
  max_distance = max(max_distance, distances[target]); 
}
average_distance /= targets.size();
```

## Voting 
```Rust
let mut formations: Map<Target, Count>;
for neighbour in GetNeighbourList() {
  formations[neighbour.squadron] += 1;
}

let mut balanced: Bool = true;
if (formations[squadron] >= squadron.force) {
  for (target: Target, count: Count) in formations {
    if (target != squadron) {
      if (count < target.force) {
        balanced = false;
        let reassignment_probability = 1 - (distances[target] / max_distance);
        let reassign = CRNG.Bernoulli(reassignment_probability);
        if (reassign) {
          squadron = target;
          break;
        }
      } else if (distances[squadron] > average_distance
              && distances[target] < average_distance) {
        balanced = false;
        let reassignment_probability = 1 - (squadron.force / formations[squadron]);
        let reassign = CRNG.Bernoulli(reassignment_probability);
        if (reassign) {
          squadron = target;
          break;
        }
      }
    }
  }
} else {
  balanced = false;
}

if (balanced) {
  state = Stage::AT_GROUND;
}
```
