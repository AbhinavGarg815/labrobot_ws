"""Who is the target?"""

from collections import deque

import numpy as np


class TargetIdentity:
    def __init__(self, short_len=20, short_period_s=0.3, long_len=40,
                 long_period_s=1.0, long_novelty=0.85, distractor_len=200,
                 distractor_period_s=0.5, reid_thresh=0.72, reid_margin=0.05,
                 reid_votes=3, add_thresh=0.70, switch_thresh=0.55,
                 switch_frames=6, top_k=3):
        self.short = deque(maxlen=short_len)
        self.short_period_s = short_period_s
        self.long = []
        self.long_len = long_len
        self.long_period_s = long_period_s
        self.long_novelty = long_novelty
        self.distractors = deque(maxlen=distractor_len)
        self.distractor_period_s = distractor_period_s
        self.reid_thresh = reid_thresh
        self.reid_margin = reid_margin
        self.reid_votes = reid_votes
        self.add_thresh = add_thresh
        self.switch_thresh = switch_thresh
        self.switch_frames = switch_frames
        self.top_k = top_k
        self.reset()

    def reset(self):
        self.short.clear()
        self.long = []
        self.distractors.clear()
        self.target_id = None
        self._last_short_t = -1e9
        self._last_long_t = -1e9
        self._last_distractor_t = {}
        self._votes = {}
        self._low_run = 0

    @property
    def has_model(self):
        return len(self.short) + len(self.long) > 0

    def gallery(self):
        g = list(self.short) + self.long
        return np.stack(g) if g else None

    def score(self, feat):
        """Mean of the top-k cosine similarities to the target memory."""
        g = self.gallery()
        if g is None or feat is None:
            return None
        sims = g @ feat
        k = min(self.top_k, sims.size)
        return float(np.mean(np.sort(sims)[-k:]))

    def impostor_score(self, feat):
        if not self.distractors or feat is None:
            return 0.0
        return float(np.max(np.stack(self.distractors) @ feat))

    def lock(self, track_id):
        """Operator selected this track; start a fresh model."""
        self.reset()
        self.target_id = track_id

    def _remember_target(self, feat, t):
        if t - self._last_short_t >= self.short_period_s:
            self.short.append(feat.copy())
            self._last_short_t = t
        if t - self._last_long_t >= self.long_period_s:
            self._last_long_t = t
            if not self.long:
                self.long.append(feat.copy())
                return
            sims = np.stack(self.long) @ feat
            if float(sims.max()) < self.long_novelty:
                self.long.append(feat.copy())
                if len(self.long) > self.long_len:
                    L = np.stack(self.long)
                    S = L @ L.T
                    np.fill_diagonal(S, -1.0)
                    # drop the entry most similar to some other entry
                    self.long.pop(int(np.argmax(S.max(axis=1))))

    def _remember_distractor(self, track_id, feat, t):
        if t - self._last_distractor_t.get(track_id, -1e9) >= self.distractor_period_s:
            self.distractors.append(feat.copy())
            self._last_distractor_t[track_id] = t

    def step(self, tracks, t, plausible=None, alive_ids=None):
        """Update with this frame's TRACKED tracks."""
        by_id = {tr.id: tr for tr in tracks}
        if self.target_id is not None and alive_ids is not None and \
                self.target_id not in alive_ids and self.target_id not in by_id:
            self.release()
        if self.target_id is not None and self.target_id in by_id:
            tgt = by_id[self.target_id]
            event = self._follow_step(tgt, by_id, t)
            if event != 'switch':
                for tr in tracks:
                    if tr.id != self.target_id and tr.det.get('clean') \
                            and tr.det.get('feat') is not None:
                        self._remember_distractor(tr.id, tr.det['feat'], t)
                return event, self.target_id
            self.target_id = None
            self._votes = {}
            return 'switch', None
        if self.target_id is not None:
            return 'missing', None
        return self._search_step(tracks, t, plausible)

    def _follow_step(self, tgt, by_id, t):
        f = tgt.det.get('feat')
        if f is None or not tgt.det.get('clean'):
            return 'occluded'
        if not self.has_model:
            self._remember_target(f, t)
            return 'tracking'
        s = self.score(f)
        if s >= self.add_thresh:
            self._low_run = 0
            self._remember_target(f, t)
            return 'tracking'
        if s < self.switch_thresh:
            self._low_run += 1
            if self._low_run >= self.switch_frames:
                # Is someone else now a better match?
                best_other = max((self.score(o.det['feat']) or -1.0
                                  for o in by_id.values()
                                  if o.id != tgt.id and o.det.get('feat') is not None),
                                 default=-1.0)
                if best_other > s + 0.10 and best_other >= self.switch_thresh:
                    self._low_run = 0
                    return 'switch'
        return 'tracking'

    def _search_step(self, tracks, t, plausible):
        if not self.has_model:
            return 'searching', None
        seen = set()
        best = None
        for tr in tracks:
            f = tr.det.get('feat')
            if not tr.confirmed or f is None or not tr.det.get('clean'):
                continue
            seen.add(tr.id)
            s = self.score(f)
            imp = self.impostor_score(f)
            ok = s >= self.reid_thresh and s - imp >= self.reid_margin and \
                (plausible is None or plausible(tr))
            if ok:
                self._votes[tr.id] = self._votes.get(tr.id, 0) + 1
                if self._votes[tr.id] >= self.reid_votes and \
                        (best is None or s > best[1]):
                    best = (tr.id, s)
            else:
                self._votes[tr.id] = 0
        for k in list(self._votes):
            if k not in seen:
                del self._votes[k]
        if best is not None:
            self.target_id = best[0]
            self._votes = {}
            self._low_run = 0
            return 'reacquired', best[0]
        return 'searching', None

    def release(self):
        self.target_id = None
        self._votes = {}
