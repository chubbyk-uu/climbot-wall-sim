# Historical legacy-contract unit fixtures

These two source snapshots and exact reviewed edits exercise the immutable SHA256
allowlist in `freeze_suction_baseline.verify_legacy_inputs`. They are test text,
not runtime models or a restored legacy launch. The other contract paths use
identical synthetic bytes: this unit checks the identity guard, not physics.

S0 source hashes and approved GUI/UDP hashes are checked before use. Snapshots
were recovered from local historical Git blobs; tests need neither private
`results/` data nor the current main files to resemble the retired robot.
