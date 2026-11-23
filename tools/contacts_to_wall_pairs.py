"""Plan-independent revisit contacts as a wall_pairs set, so `wall_pairs.py measure` runs on them for any trajectory.

  python tools/contacts_to_wall_pairs.py <contacts.pkl> <sample.npz> <trajectory> <out.npz>
         [--local-shift X Y Z] [--node-ties node_ties.npz]

The contacts are the 852 revisit faces of the earlier Livox 4 study (doubles/codex/lever_contacts.pkl, found by its
wall_atlas without a plan, over four storeys): a list of dicts with the returns of the two visits (`ia`, `ib`: indices
into the sample's points, `ga`, `gb`: the cell of each return), the cells both visits share (`shared`), the face's
normal `n` in the frame of the trajectory they were made on, and its `level` (storey). `sample.npz` holds `t` (point
time, s) and `xyz` (the placed points, same frame) of that result; <trajectory> is that result's folder (the nodes the
points were placed with). The pickle is read as is: only open one made by your own tools.

Conversion, as the study measured it: every return is stored relative to its node (the node pose inverse applied; with
--local-shift added to the node-relative position, the lever arm correction between the arm the points were placed with
and the one the trajectories to measure were run with). Every shared cell is one pair of visits (the earlier visit's
returns in that cell, the later visit's), the contact's normal is stored with the node pose it was made in, and a
contact's gap is the median of its cells' gaps (`pair_contact`, as `wall_pairs.py measure` reads it). The contact's own
time ranges (`contact_t`, over all its returns, not by cell) decide early / across, and `contact_level` gives one class
per storey. `node_ties.npz` (ns, idx) optionally gives the node of returns whose time ties a node's exactly; without
it a return belongs to the node strictly before it (wall_pairs.node_of).

Needs numpy.
"""
import argparse
import json
import pickle
import sys
from pathlib import Path

import numpy as np

import wall_pairs as wp


def convert(contacts_path, sample_path, trajectory, out, local_shift=(0.0, 0.0, 0.0), node_ties=None, log=print):
    with open(contacts_path, "rb") as fp:
        qs = pickle.load(fp)
    with np.load(sample_path) as z:
        t, x = np.asarray(z["t"], float), np.asarray(z["xyz"], float)
    tr = wp.load_trajectory(trajectory)
    T = tr.ns * 1e-9
    node = wp.node_of(t, tr.ns)
    if node_ties:
        with np.load(node_ties) as z:
            keys, values = z["ns"], z["idx"]
        ns = np.rint(t * 1e9).astype(np.int64)
        k = np.searchsorted(keys, ns).clip(max=len(keys) - 1)
        eq = ns == keys[k]
        node[eq] = values[k[eq]]
    if node.min() < 0 or node.max() >= len(T):
        raise ValueError("returns of the sample lie before the first node or after the last: another trajectory?")
    shift = np.asarray(local_shift, float)
    ret_t, ret_node, ret_local, ret_world = [], [], [], []
    start, count, vpiece = [], [], []
    pair_a, pair_b, pair_contact, pair_anchor = [], [], [], []
    contact_t, level, normals, centers, ids = [], [], [], [], []
    at = 0
    for k, q in enumerate(qs):
        ia, ib = np.asarray(q["ia"]), np.asarray(q["ib"])
        anchor = int(np.searchsorted(T, np.median(t[ia])))
        anchor = min(anchor, len(T) - 1)
        contact_t.append((t[ia].min(), t[ia].max(), t[ib].min(), t[ib].max()))
        level.append(int(q["level"]))
        normals.append(np.asarray(q["n"], float))
        centers.append(np.asarray(q["c"], float)[:2])
        ids.append(k)
        for cell in q["shared"]:
            va = ia[np.asarray(q["ga"]) == cell]
            vb = ib[np.asarray(q["gb"]) == cell]
            if len(va) == 0 or len(vb) == 0:
                raise ValueError(f"contact {k}: shared cell {cell} has no returns in one visit")
            pair_a.append(len(start))
            pair_b.append(len(start) + 1)
            pair_contact.append(k)
            pair_anchor.append(anchor)
            for v in (va, vb):
                nd = node[v]
                loc = wp.to_local(x[v], nd, tr) + shift
                ret_t.append(t[v])
                ret_node.append(nd)
                ret_local.append(loc)
                start.append(at)
                count.append(len(v))
                vpiece.append(k)
                at += len(v)
    if not pair_a:
        raise ValueError("no contact has a shared cell")
    node_all = np.concatenate(ret_node)
    local = np.concatenate(ret_local)
    world = wp.place(local, node_all, tr)   # the stored returns on the trajectory they were taken from, with the shift applied
    ct = np.array(contact_t)
    anchors = np.array(pair_anchor)
    meta = dict(origin_result=str(Path(trajectory).resolve()), selection=f"plan-independent contacts ({len(qs)}) from {Path(contacts_path).name}; "
                f"a contact's gap is the median of its {len(pair_a)} shared cells' gaps", contacts=len(qs), contacts_file=str(Path(contacts_path).resolve()),
                sample_file=str(Path(sample_path).resolve()), local_shift_m=[float(v) for v in shift], node_ties=str(node_ties) if node_ties else None,
                nodes=len(tr.ns), pieces=len(qs), visits=len(start), pairs=len(pair_a), returns=int(at), height_m=None)
    np.savez_compressed(
        out, meta=np.array(json.dumps(meta)), node_ns=tr.ns, t=np.concatenate(ret_t), node=node_all.astype(np.int32), local=local, world=world,
        visit_start=np.array(start), visit_count=np.array(count), visit_piece=np.array(vpiece), visit_side=np.zeros(len(start), int),
        visit_t0=np.array([tv.min() for tv in ret_t]), visit_t1=np.array([tv.max() for tv in ret_t]),
        piece_table_id=np.array(ids), piece_xy=np.array(centers), piece_normal=np.array(normals),
        pair_a=np.array(pair_a), pair_b=np.array(pair_b), pair_origin_gap=np.full(len(pair_a), np.nan), pair_anchor=anchors,
        pair_anchor_rot=tr.rot[anchors], pair_contact=np.array(pair_contact), contact_t=ct, contact_level=np.array(level))
    log(f"converted {out}: {len(qs)} contacts, {len(pair_a)} shared cells, {int(at)} stored returns; storeys "
        + ", ".join(f"{lv}: {int((np.array(level) == lv).sum())}" for lv in sorted(set(level))))
    return meta


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("contacts")
    ap.add_argument("sample")
    ap.add_argument("trajectory")
    ap.add_argument("out")
    ap.add_argument("--local-shift", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"))
    ap.add_argument("--node-ties")
    a = ap.parse_args(argv)
    try:
        convert(a.contacts, a.sample, a.trajectory, a.out, a.local_shift, a.node_ties)
    except (ValueError, FileNotFoundError) as e:
        print(f"contacts_to_wall_pairs: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
