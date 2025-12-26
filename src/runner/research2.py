"""Generate a robot trajectory that traces a rendered text string.

Reads configuration from YAML, renders text to an image, converts to binary,
extracts contours, connects contours with quintic interpolation, and
writes a CSV with (x,y,z,0,0,0) rows suitable for playback.

run: python src/runner/research2.py --config configs/research2_config.yaml

"""
from __future__ import annotations
import yaml
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os
import logging
from typing import List, Tuple

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


def tpoly_segment(p0: np.ndarray, pf: np.ndarray, N: int) -> np.ndarray:
    num = max(2, N + 1)
    t = np.linspace(0, 1, num)
    s = 6 * t**5 - 15 * t**4 + 10 * t**3
    return (p0[np.newaxis, :] + (pf - p0)[np.newaxis, :] * s[:, np.newaxis])


def moore_boundary_tracing(mask: np.ndarray) -> List[np.ndarray]:
    """Extract ordered boundary coordinates for each connected component using a
    simple Moore-like neighbor tracing algorithm.

    Returns list of arrays with shape (L,2) for each contour (row,col).
    """
    from scipy import ndimage
    labeled, n = ndimage.label(mask)
    contours = []
    for label in range(1, n+1):
        comp = (labeled == label)
        # find true boundary pixels: pixel is 1 and has at least one 0 neighbor
        pad = np.pad(comp, pad_width=1, mode='constant', constant_values=0)
        rows, cols = pad.shape
        boundary_pixels = []
        for r in range(1, rows-1):
            for c in range(1, cols-1):
                if pad[r, c]:
                    # if any neighbor is 0, it's a boundary pixel
                    if np.any(pad[r-1:r+2, c-1:c+2] == 0):
                        boundary_pixels.append((r, c))
        if len(boundary_pixels) == 0:
            continue
        comp_set = set(boundary_pixels)
        start = min(comp_set)
        boundary = []
        neigh = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
        curr = start
        prev_dir = 7
        while True:
            boundary.append((curr[0]-1, curr[1]-1))
            found = False
            for k in range(8):
                idx = (prev_dir + 1 + k) % 8
                nr = curr[0] + neigh[idx][0]
                nc = curr[1] + neigh[idx][1]
                if (nr, nc) in comp_set:
                    curr = (nr, nc)
                    prev_dir = (idx + 4) % 8
                    found = True
                    break
            if not found:
                break
            if curr == start:
                break
        contours.append(np.array(boundary, dtype=float))

    # Also detect enclosed background components (holes) and extract their boundaries
    inv_labeled, inv_n = ndimage.label(np.logical_not(mask))
    for label in range(1, inv_n+1):
        bg_comp = (inv_labeled == label)
        # skip background regions touching the image border (these are external background)
        if np.any(bg_comp[0, :]) or np.any(bg_comp[-1, :]) or np.any(bg_comp[:, 0]) or np.any(bg_comp[:, -1]):
            continue
        pad = np.pad(bg_comp, pad_width=1, mode='constant', constant_values=0)
        rows, cols = pad.shape
        boundary_pixels = []
        for r in range(1, rows-1):
            for c in range(1, cols-1):
                if pad[r, c]:
                    # if any neighbor is 0, it's a boundary pixel for the hole (adjacent to text)
                    if np.any(pad[r-1:r+2, c-1:c+2] == 0):
                        boundary_pixels.append((r, c))
        if len(boundary_pixels) == 0:
            continue
        comp_set = set(boundary_pixels)
        start = min(comp_set)
        boundary = []
        neigh = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
        curr = start
        prev_dir = 7
        while True:
            # convert back to image coords (remove padding)
            boundary.append((curr[0]-1, curr[1]-1))
            found = False
            for k in range(8):
                idx = (prev_dir + 1 + k) % 8
                nr = curr[0] + neigh[idx][0]
                nc = curr[1] + neigh[idx][1]
                if (nr, nc) in comp_set:
                    curr = (nr, nc)
                    prev_dir = (idx + 4) % 8
                    found = True
                    break
            if not found:
                break
            if curr == start:
                break
        contours.append(np.array(boundary, dtype=float))
    return contours


def binarize_image(img: Image.Image, threshold: int = None) -> np.ndarray:
    gray = img.convert('L')
    arr = np.array(gray).astype(float)
    if threshold is None:
        hist, bins = np.histogram(arr.ravel(), bins=256)
        total = arr.size
        current_max, threshold_value = 0, 0
        sum_total = np.dot(np.arange(256), hist)
        sumB = 0.0
        wB = 0.0
        wF = 0.0
        for i in range(256):
            wB += hist[i]
            if wB == 0:
                continue
            wF = total - wB
            if wF == 0:
                break
            sumB += i * hist[i]
            mB = sumB / wB
            mF = (sum_total - sumB) / wF
            between = wB * wF * (mB - mF) ** 2
            if between >= current_max:
                current_max = between
                threshold_value = i
        threshold = threshold_value
    binary = (arr > threshold)
    binary = np.logical_not(binary).astype(np.uint8)
    return binary


def generate(cfg_path: str) -> str:
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    text = cfg.get('text_string', '197x张三')
    font_size = int(cfg.get('font_size', 200))
    font_path = cfg.get('font_path', None)
    image_size = tuple(cfg.get('image_size', [1000, 400]))
    threshold = cfg.get('threshold', None)
    interp_density = float(cfg.get('interp_density', 0.5))
    z_height = float(cfg.get('z_height', 200))
    start_point = np.array(cfg.get('start_point', [302, 0, 558, 0, 0, 0]), dtype=float)
    output_path = cfg.get('output_path', 'data/data_output/research2_trajectory_py.csv')

    if font_path is not None and os.path.exists(font_path):
        font = ImageFont.truetype(font_path, font_size)
    else:
        try:
            font = ImageFont.truetype("arial.ttf", font_size)
        except Exception:
            font = ImageFont.load_default()
            logging.warning('Using default bitmap font; rendering quality may vary.')

    import matplotlib.pyplot as plt
    from matplotlib import rcParams

    # Prefer a font that supports CJK if available (user can configure font_path)
    if font_path is not None and os.path.exists(font_path):
        rc_font = {'font.family': 'sans-serif', 'font.sans-serif': [font_path]}
        plt.rcParams.update(rc_font)

    # Create a Matplotlib figure with explicit size and DPI so pixel output matches image_size
    dpi = cfg.get('render_dpi', 100)
    fig = plt.figure(figsize=(image_size[0] / dpi, image_size[1] / dpi), dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')

    # Use matplotlib text rendering centered in the canvas
    ax.text(0.5, 0.5, text, fontsize=font_size, ha='center', va='center', color='black')
    fig.canvas.draw()

    w, h = fig.canvas.get_width_height()
    # Support different matplotlib canvas implementations: try RGB, fallback to ARGB
    try:
        buf = fig.canvas.tostring_rgb()
        arr = np.frombuffer(buf, dtype='uint8').reshape((h, w, 3))
    except AttributeError:
        # Some backends expose tostring_argb(); convert ARGB -> RGB
        buf = fig.canvas.tostring_argb()
        arr_argb = np.frombuffer(buf, dtype='uint8').reshape((h, w, 4))
        arr = arr_argb[:, :, [1, 2, 3]]
    img = Image.fromarray(arr)
    plt.close(fig)

    # If the final generated image differs in size, optionally rescale to desired image_size
    if (w, h) != image_size:
        img = img.resize(image_size, resample=Image.NEAREST)


    downscale = float(cfg.get('downscale', 1.0))
    if downscale and downscale > 0 and downscale < 1.0:
        newsize = (max(1, int(image_size[0] * downscale)), max(1, int(image_size[1] * downscale)))
        img = img.resize(newsize, resample=Image.NEAREST)
        logging.info(f'Downscaled rendered image to {newsize} (factor {downscale})')

    # Binarize using the existing Otsu-like function
    bin_img = binarize_image(img, threshold=threshold)

    # Optional: apply small morphological opening to remove thin bridges and restore small holes
    # This helps recover thin inner holes (e.g., 0, 8) without increasing downscale
    from scipy import ndimage as _nd
    morph_open = bool(cfg.get('morph_open', True))
    if morph_open:
        opened = _nd.binary_opening(bin_img, structure=_nd.generate_binary_structure(2,1))
        logging.info(f'Applied morphological opening to binary image (morph_open={morph_open})')
        bin_img = opened.astype(np.uint8)


    boundary_method = cfg.get('boundary_method', 'moore')
    contours = []
    if boundary_method == 'find_contours':
        try:
            from skimage import measure
        except Exception as e:
            raise RuntimeError('scikit-image is required for find_contours: pip install scikit-image') from e
        smoothed = bin_img.astype(np.uint8)
        raw_contours = measure.find_contours(smoothed, level=0.5)
        for rc in raw_contours:
            contours.append(np.array(rc, dtype=float))
        if len(contours) == 0:
            raise RuntimeError('No contours found in binary image (after find_contours)')
    elif boundary_method == 'textpath':
        # Extract vector outlines using Matplotlib TextPath for each glyph (vector outlines, not raster boundaries)
        try:
            from matplotlib.textpath import TextPath
            from matplotlib.font_manager import FontProperties
        except Exception as e:
            raise RuntimeError('matplotlib is required for textpath boundary extraction') from e
        # Build glyph paths and convert to contour arrays in (row,col) order
        fp = FontProperties(fname=font_path, size=font_size) if (font_path and os.path.exists(font_path)) else FontProperties(size=font_size)
        verts_all = []
        current_x = 0.0
        widths = []
        glyphs = []
        for ch in text:
            tp = TextPath((0, 0), ch, prop=fp)
            verts = tp.vertices.copy()
            # shift horizontally by current_x
            verts[:, 0] += current_x
            current_x += tp.get_extents().width
            glyphs.append(verts)
            widths.append(tp.get_extents().width)
        total_width = sum(widths) if widths else 0.0
        # center horizontally and vertically in image coordinate
        center_x = total_width / 2.0
        img_h = image_size[1]
        img_center_y = img_h / 2.0
        for verts in glyphs:
            # TextPath coords: (x,y) with y upwards; convert to image row (y downward) and center
            xs = verts[:, 0] - center_x
            ys = verts[:, 1]
            # flip Y to image rows and center
            rows = img_center_y - ys
            cols = xs
            contours.append(np.column_stack((rows, cols)))
        if len(contours) == 0:
            raise RuntimeError('No contours generated by TextPath')
    else:
        # Use the Moore neighborhood boundary tracing implemented above (closest to bwboundaries behavior)
        contours = moore_boundary_tracing(bin_img)
        if len(contours) == 0:
            raise RuntimeError('No contours found in binary image (after moore_boundary_tracing)')


    # --- Improve contour ordering and add travel lift to avoid drawing travel moves ---
    # center columns using all contours then sort contours left-to-right so glyphs are traced in reading order
    all_cols = np.concatenate([ct[:, 1] for ct in contours])
    col_center = (all_cols.max() + all_cols.min()) / 2.0

    # Group contours into clusters in X (one cluster per glyph) to keep glyph parts together.
    # Estimate typical character width from text length and overall span.
    span_x = (all_cols.max() - all_cols.min())
    est_char_w = span_x / max(1, len(text))
    cluster_thresh = float(cfg.get('cluster_x_thresh', max(5.0, est_char_w * 0.6)))

    # Prefer a stable left-to-right grouping by dividing the total X span into len(text) bins
    centroids = np.array([float(np.mean(c[:, 1])) for c in contours])
    order = np.argsort(centroids)

    n_chars = max(1, len(text))

    # Prefer glyph-aware binning using TextPath widths if available (more accurate for non-uniform glyph widths)
    try:
        from matplotlib.textpath import TextPath
        from matplotlib.font_manager import FontProperties
        fp = FontProperties(fname=font_path, size=font_size) if (font_path and os.path.exists(font_path)) else FontProperties(size=font_size)
        widths = []
        for ch in text:
            tp = TextPath((0, 0), ch, prop=fp)
            widths.append(tp.get_extents().width)
        total_w = sum(widths)
        if total_w > 0:
            centers = np.cumsum([0.0] + widths[:-1]) + np.array(widths) / 2.0
            # scale centers to image coordinate span
            span = all_cols.max() - all_cols.min()
            scaled_centers = all_cols.min() + (centers / total_w) * span
            # assign each contour to nearest glyph center
            bin_idx = np.array([int(np.argmin(np.abs(c - scaled_centers))) for c in centroids])
        else:
            raise Exception('total glyph width zero')
    except Exception:
        # fallback: uniform bins
        bins = np.linspace(all_cols.min(), all_cols.max(), num=n_chars + 1)
        bin_idx = np.clip(np.digitize(centroids, bins) - 1, 0, n_chars - 1)

    # Build initial clusters by character bin (ensures left-to-right ordering)
    clusters = [[] for _ in range(n_chars)]
    for i in order:
        clusters[bin_idx[i]].append(i)
    # Detect which contours are holes by sampling the binary mask at their centroid (more robust later)
    hole_flags: List[bool] = []
    for i, c in enumerate(contours):
        r_mean = int(round(np.mean(c[:, 0])))
        c_mean = int(round(np.mean(c[:, 1])))
        if 0 <= r_mean < bin_img.shape[0] and 0 <= c_mean < bin_img.shape[1]:
            hole_flags.append(bool(bin_img[r_mean, c_mean] == 0))
        else:
            hole_flags.append(False)

    # Build mapping contour -> cluster id
    contour_to_cluster = [-1] * len(contours)
    for cid, cl in enumerate(clusters):
        for idx in cl:
            contour_to_cluster[idx] = cid

    # Attach hole contours to nearest non-hole contour within the same bin; otherwise find nearest non-hole globally
    non_holes = [j for j in range(len(contours)) if not hole_flags[j]]
    for i, is_hole in enumerate(hole_flags):
        if not is_hole:
            continue
        cid = contour_to_cluster[i]
        # if cid has a non-hole, keep it
        if cid >= 0:
            cl = clusters[cid]
            if any(not hole_flags[j] for j in cl):
                continue
        # try to find nearest non-hole in same bin
        same_bin_candidates = [j for j in clusters[cid] if not hole_flags[j]] if cid >= 0 else []
        if len(same_bin_candidates) > 0:
            j_min = min(same_bin_candidates, key=lambda j: abs(centroids[i] - centroids[j]))
            contour_to_cluster[i] = contour_to_cluster[j_min]
            continue
        # fallback: nearest non-hole globally
        if len(non_holes) == 0:
            continue
        j_min = min(non_holes, key=lambda j: abs(centroids[i] - centroids[j]))
        contour_to_cluster[i] = contour_to_cluster[j_min]

    # Log holes and their assigned clusters
    for i, is_hole in enumerate(hole_flags):
        if not is_hole:
            continue
        logging.info(f'hole contour idx={i} centroid={centroids[i]:.2f} assigned_cluster={contour_to_cluster[i]} pts={len(contours[i])}')

    # Rebuild clusters from contour_to_cluster map; ensure bins correspond to characters and keep empty bins
    new_clusters = [[] for _ in range(n_chars)]
    for idx in order:
        cid = contour_to_cluster[idx]
        if cid is None or cid < 0 or cid >= n_chars:
            # put any unassigned at the nearest bin by centroid
            cid = int(np.clip(np.digitize(centroids[idx], bins) - 1, 0, n_chars - 1))
        new_clusters[cid].append(idx)
    # Keep bins as-is (do not drop empty bins) to preserve stable mapping to characters
    clusters = new_clusters

    # Log mapping from cluster(index) -> (count, mean centroid) for debugging
    for ci, cl in enumerate(clusters):
        if len(cl) == 0:
            logging.info(f'cluster idx={ci} (char_pos={ci}) count=0')
        else:
            mean_x = float(np.mean([centroids[i] for i in cl]))
            logging.info(f'cluster idx={ci} (char_pos={ci}) count={len(cl)} centroid_x={mean_x:.2f}')
    logging.info(f'Binning: n_chars={n_chars}, bins_kept={len(clusters)}')

    # Determine draw order of clusters by their centroid X (left-to-right physical order)
    cluster_centroids = []
    for ci, cl in enumerate(clusters):
        if len(cl) == 0:
            cluster_centroids.append(float('nan'))
        else:
            cluster_centroids.append(float(np.mean([centroids[i] for i in cl])))

    # Map each cluster to the nearest glyph index (using glyph centers if available)
    glyph_centers = None
    try:
        from matplotlib.textpath import TextPath
        from matplotlib.font_manager import FontProperties
        fp = FontProperties(fname=font_path, size=font_size) if (font_path and os.path.exists(font_path)) else FontProperties(size=font_size)
        widths = [TextPath((0,0), ch, prop=fp).get_extents().width for ch in text]
        total_w = sum(widths)
        if total_w > 0:
            centers = np.cumsum([0.0] + widths[:-1]) + np.array(widths) / 2.0
            span = all_cols.max() - all_cols.min()
            glyph_centers = all_cols.min() + (centers / total_w) * span
    except Exception:
        glyph_centers = None

    # For each cluster compute its centroid and assign to nearest glyph
    cluster_to_glyph = {}
    for ci, cl in enumerate(clusters):
        if len(cl) == 0:
            continue
        cl_cent = float(np.mean([centroids[i] for i in cl]))
        if glyph_centers is not None:
            gi = int(np.argmin(np.abs(cl_cent - glyph_centers)))
        else:
            # fallback: map by sorted cluster centroid order
            gi = ci
        cluster_to_glyph[ci] = gi

    # Build mapping glyph index -> list of clusters (preserve cluster internal ordering by centroid)
    glyph_to_clusters = {i: [] for i in range(max(1, len(text)))}
    for ci in range(len(clusters)):
        # fallback: if cluster not mapped to a glyph, map by centroid into uniform bins
        if ci in cluster_to_glyph:
            gi = cluster_to_glyph[ci]
        else:
            fallback_bins = np.linspace(all_cols.min(), all_cols.max(), num=n_chars + 1)
            cid_guess = int(np.clip(np.digitize(cluster_centroids[ci] if not np.isnan(cluster_centroids[ci]) else all_cols.min(), fallback_bins) - 1, 0, n_chars - 1))
            gi = cid_guess
        glyph_to_clusters[gi].append(ci)

    # Determine drawing order: glyph indices ascending (0..len(text)-1)
    new_contours: List[np.ndarray] = []
    contour_cluster_idx: List[int] = []
    draw_order = []
    for gi in sorted(glyph_to_clusters.keys()):
        clist = glyph_to_clusters[gi]
        # sort clusters by their centroid so multi-cluster glyphs are drawn consistently
        clist_sorted = sorted(clist, key=lambda c: float(np.mean([centroids[i] for i in clusters[c]])) if len(clusters[c])>0 else 0.0)
        for ci in clist_sorted:
            draw_order.append((gi, ci))
            cl = clusters[ci]
            outer = [i for i in cl if not hole_flags[i]]
            holes = [i for i in cl if hole_flags[i]]
            outer_sorted = sorted(outer, key=lambda i: -float(np.ptp(contours[i][:, 0]) * np.ptp(contours[i][:, 1])))
            assigned_holes = set()
            for o in outer_sorted:
                new_contours.append(contours[o])
                contour_cluster_idx.append(ci)
                nearby_holes = sorted([h for h in holes if h not in assigned_holes], key=lambda h: abs(centroids[h] - centroids[o]))
                for h in nearby_holes:
                    new_contours.append(contours[h])
                    contour_cluster_idx.append(ci)
                    assigned_holes.add(h)
            for h in holes:
                if h not in assigned_holes:
                    new_contours.append(contours[h])
                    contour_cluster_idx.append(ci)

    contours = new_contours

    logging.info(f'Draw cluster mapping glyph->clusters: { {gi: glyph_to_clusters[gi] for gi in sorted(glyph_to_clusters.keys())} }')
    logging.info(f'Draw order (glyph,cluster): {draw_order}')

    # Log cluster summary for debugging ordering issues
    try:
        import math
        cluster_summaries = []
        for ci, cl in enumerate(clusters):
            xs = [centroids[i] for i in cl]
            cluster_summaries.append((ci, len(cl), float(np.mean(xs)), float(np.ptp(xs) if len(xs)>0 else 0.0)))
        for s in cluster_summaries:
            logging.info(f'cluster idx={s[0]} count={s[1]} centroid_x={s[2]:.2f} span={s[3]:.2f}')
    except Exception:
        pass

    travel_z = float(cfg.get('travel_z', max(10.0, z_height * 0.1)))
    total_contour_pts = 0
    total_interp_pts = 0
    interp_details = []

    def to_3d(pt2d, z):
        return np.array([pt2d[0], pt2d[1], z], dtype=float)

    # Build a sequence of 3D drawing points (at drawing height z_height). Between contours we will lift to z_height+travel_z,
    # move in XY at the raised height, then lower to drawing z to avoid drawing connecting lines.
    draw_pts = []
    prev_end_2d = None

    logging.info('Contour summary (idx, centroid_col, pts):')
    for idx, c in enumerate(contours):
        logging.info(f'  idx={idx} centroid_col={np.mean(c[:,1]):.2f} pts={len(c)}')

    for k, ct in enumerate(contours):
        ct_arr = np.array(ct)
        decimate = int(cfg.get('contour_decimate', 1))
        if decimate > 1:
            ct_arr = ct_arr[::decimate]
        if len(ct_arr) == 0:
            continue
        # center columns
        ct_arr[:, 1] = ct_arr[:, 1] - col_center

        # orient contour to minimize jump from previous end
        if prev_end_2d is not None and len(ct_arr) > 1:
            d0 = np.linalg.norm(ct_arr[0] - prev_end_2d)
            d1 = np.linalg.norm(ct_arr[-1] - prev_end_2d)
            if d1 < d0:
                ct_arr = ct_arr[::-1]

        start2 = ct_arr[0]
        end2 = ct_arr[-1]
        total_contour_pts += len(ct_arr)

        # If there is a previous contour, decide whether to do a full lifted travel or a short on-plane connector
        if prev_end_2d is not None:
            gap = np.linalg.norm(start2 - prev_end_2d)
            no_lift_thresh = float(cfg.get('no_lift_threshold', 6.0))
            same_cluster = False
            if 'contour_cluster_idx' in locals():
                same_cluster = (contour_cluster_idx[k] == contour_cluster_idx[k-1]) if k-1 >= 0 else False
            # Force no lift for contours in the same glyph cluster
            if gap <= no_lift_thresh or same_cluster:
                # small gap or same glyph: connect directly at drawing Z (avoid unnecessary lift, keeps holes and parts continuous)
                n_conn = max(2, int(np.floor(gap / 2.0)))
                conn = tpoly_segment(to_3d(prev_end_2d, z_height), to_3d(start2, z_height), n_conn)
                total_interp_pts += conn.shape[0]
                interp_details.append((k, float(gap), conn.shape[0]))
                draw_pts.extend(conn.tolist())
            else:
                # up from drawing Z to travel Z (or to higher start Z if present)
                p_up0 = to_3d(prev_end_2d, z_height + travel_z)
                p_up1 = to_3d(start2, z_height + travel_z)
                # small raise/lower steps
                n_up = max(2, int(np.ceil(travel_z / 2.0)))
                raise_seg = tpoly_segment(to_3d(prev_end_2d, z_height), p_up0, n_up)
                # cap travel interpolation points to avoid very long slow travel traces
                dist_up = np.linalg.norm(p_up0[:2] - p_up1[:2])
                max_travel = int(cfg.get('max_travel_points', 20))
                travel_n = min(max_travel, max(2, int(np.floor(dist_up / 2.0))))
                travel_seg = tpoly_segment(p_up0, p_up1, travel_n)
                lower_seg = tpoly_segment(p_up1, to_3d(start2, z_height), n_up)
                total_interp_pts += raise_seg.shape[0] + travel_seg.shape[0] + lower_seg.shape[0]
                interp_details.append((k, float(gap), raise_seg.shape[0]+travel_seg.shape[0]+lower_seg.shape[0]))
                draw_pts.extend(raise_seg.tolist())
                draw_pts.extend(travel_seg.tolist())
                draw_pts.extend(lower_seg.tolist())
        # append the contour points at drawing height
        for p in ct_arr.tolist():
            draw_pts.append(to_3d(p, z_height).tolist())

        prev_end_2d = end2

    logging.info(f'contours: {len(contours)}, total_contour_pts={total_contour_pts}, total_interp_pts={total_interp_pts}')
    for d in interp_details[:10]:
        logging.info(f'interp seg idx={d[0]} total_pts={d[2]}')

    draw_pts = np.array(draw_pts, dtype=float)

    # Begin and end traces: move from start_point to first drawing point using a lifted approach to avoid drawing long connecting lines
    if draw_pts.shape[0] == 0:
        raise RuntimeError('No drawn points generated from contours')

    first_pt = draw_pts[0]
    last_pt = draw_pts[-1]

    def lifted_connect(p_from3, p_to3):
        # returns array of points: raise, travel, lower
        # choose an "up" Z that is at least the configured travel height above drawing height,
        # but do not further raise if the from/to points are already above it.
        desired_up_z = max(z_height + travel_z, p_from3[2], p_to3[2])
        p_from_up = np.array([p_from3[0], p_from3[1], desired_up_z], dtype=float)
        p_to_up = np.array([p_to3[0], p_to3[1], desired_up_z], dtype=float)
        n_up = max(2, int(np.ceil(abs(desired_up_z - p_from3[2]) / 2.0)))
        seg_raise = tpoly_segment(p_from3, p_from_up, n_up)
        # cap travel interpolation points using configuration to keep travel fast
        dist_up = np.linalg.norm(p_from_up[:2] - p_to_up[:2])
        max_travel = int(cfg.get('max_travel_points', 20))
        travel_n = min(max_travel, max(2, int(np.floor(dist_up / 2.0))))
        seg_travel = tpoly_segment(p_from_up, p_to_up, travel_n)
        seg_lower = tpoly_segment(p_to_up, p_to3, max(2, int(np.ceil(abs(desired_up_z - p_to3[2]) / 2.0))))
        return np.vstack((seg_raise, seg_travel, seg_lower))

    sp3 = np.array(start_point[:3], dtype=float)
    # ensure starting point is at same z as expected
    sp3 = np.array([sp3[0], sp3[1], sp3[2]], dtype=float)

    # --- BEGIN: gentler initial approach ---
    # Instead of directly descending to drawing Z over a long travel, first approach to a
    # near-drawing arrival height (z_height + initial_arrival_margin), then perform a short
    # vertical drop to drawing Z at the exact start XY. This avoids plunging the arm too low early.
    arrival_margin = float(cfg.get('initial_arrival_margin', 20.0))
    arrival_z = float(z_height + arrival_margin)
    # approach point shares the first point XY but with arrival_z
    p_arrival = np.array([first_pt[0], first_pt[1], arrival_z], dtype=float)
    dist_approach = np.linalg.norm(sp3[:2] - p_arrival[:2])
    max_travel = int(cfg.get('max_travel_points', 20))
    n_approach = min(max_travel, max(2, int(np.floor(dist_approach / 2.0))))
    begin_approach = tpoly_segment(sp3, p_arrival, n_approach)
    # short vertical lower from arrival_z to drawing z (small number of points)
    n_lower = max(2, int(np.ceil(arrival_margin / 2.0)))
    begin_lower = tpoly_segment(p_arrival, np.array([first_pt[0], first_pt[1], z_height], dtype=float), n_lower)
    begin_seg = np.vstack((begin_approach, begin_lower))
    logging.info(f'Begin approach: arrival_z={arrival_z}, approach_pts={begin_approach.shape[0]}, lower_pts={begin_lower.shape[0]}')
    # --- END: gentler initial approach ---

    end_seg = lifted_connect(last_pt, sp3)

    # combine begin -> draw_pts -> end
    out3 = np.vstack((begin_seg, draw_pts, end_seg))

    # convert to output format: (x=row, y=col_centered, z, 0,0,0)
    out = np.hstack((out3, np.zeros((out3.shape[0], 3))))

    out_dir = os.path.dirname(output_path)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    
    def fmt_val(v: float, ndigits: int = 6) -> str:
        # Treat very near-integers as integers for cleaner output
        if abs(v - round(v)) < 1e-9:
            return str(int(round(v)))
        # Format with fixed 6 decimal places then trim trailing zeros (matches the clean file)
        s = f"{v:.{ndigits}f}".rstrip('0').rstrip('.')
        return s if s != '' else '0'

    import csv
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f, lineterminator='\r\n')
        for row in out:
            writer.writerow([fmt_val(float(x)) for x in row])
    logging.info(f'Wrote trajectory CSV to {output_path} (csv.writer with CRLF and formatted values)')

    return output_path


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c', default='configs/research2_config.yaml')
    args = parser.parse_args()
    generate(args.config)


