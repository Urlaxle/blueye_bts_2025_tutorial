# aruco_board_actual_sim_bts_rig.py
import numpy as np

# ---- Your measured corners (meters) ----
corners_raw = {
    25: np.array([[-0.07000000029802324,  0.07000000029802322, -1.4108548039097074e-17],
                  [ 0.07000000029802324,  0.07000000029802322,  1.430625493730526e-17],
                  [ 0.07000000029802324, -0.07000000029802322,  1.410894752554046e-17],
                  [-0.07000000029802324, -0.07000000029802322, -1.4305855450861872e-17]], dtype='float32'),

    26: np.array([[0.401937716040624,  0.07179118664075092,  0.0013881201953675924],
                  [0.5419376842795891,  0.07185024496823582,  0.0013134737236670249],
                  [0.541996764153594, -0.06814973733297834,  0.001353873243651303],
                  [0.40199679591462895, -0.06820879566046324,  0.0014285197153518705]], dtype='float32'),

    27: np.array([[0.13693111910758854,  0.038510553919978784, -0.8613837109310651],
                  [0.21692963866853077,  0.038989458323330346, -0.861298666661934],
                  [0.21740857573373923, -0.041009100430776484, -0.8613291699567915],
                  [0.137410056172797, -0.041488004834128046, -0.8614142142259227]], dtype='float32'),

    28: np.array([[0.2744869754132579,  0.03821109333235176, -0.8594577048013036],
                  [0.35448688822242713,  0.038327718847857364, -0.859465636519597],
                  [0.3546035094300323, -0.041672182651637205, -0.859508920036201],
                  [0.2746035966208631, -0.04178880816714281, -0.8595009883179077]], dtype='float32'),

    29: np.array([[0.20840526655694003, -0.12024365505211826, -0.8725128456979605],
                  [0.2884038265525557, -0.11978730264240431, -0.8723650113185338],
                  [0.28886014305375096, -0.1997859969772544, -0.8723451652189287],
                  [0.2088615830581353, -0.20024234938696836, -0.8724929995983554]], dtype='float32'),
}


# ---- Helper: center of a tag from its corners ----
def center(c4x3: np.ndarray) -> np.ndarray:
    return np.mean(c4x3, axis=0)

# ---- Choose origin at tag 29 center (in measured "board" coords) ----
c29 = center(corners_raw[29])   # 3-vector

# ---- Alignment rotation to emulate the old controller frame ----
# Map: forward(-Z_meas) -> +X_ctrl,   +X_meas -> +Y_ctrl,   +Y_meas -> +Z_ctrl
# A = np.array([[ 0., 0., -1.],
#               [ 1., 0.,  0.],
#               [ 0., 1.,  0.]], dtype='float32')

# S = np.array([[ 1., 0., 0.],
#               [ 0.,-1., 0.],
#               [ 0., 0.,-1.]], dtype='float32')

S = np.array([[ 1., 0., 0.],
               [ 0.,1., 0.],
               [ 0., 0.,1.]], dtype='float32')
# Afix = S @ A
Afix = S



# ---- Build pos_board in the controller-compatible frame ----
pos_board = []
id_board_list = []

board_ids_list = [25, 26, 29]

for tid in [25, 26, 27, 28, 29]:
    # shift so tag29 center is origin, then rotate by A
    shifted = corners_raw[tid] - c29[None, :]
    rotated = (Afix @ shifted.T).T
    pos_board.append(rotated.astype('float32'))
    id_board_list.append([tid])

# numpy arrays in the shape OpenCV expects
id_board = np.array(id_board_list, dtype=np.int32)

# Optional: quick sanity print
if __name__ == "__main__":
    for i, tid in enumerate([25,26,27,28,29]):
        ctr = pos_board[i].mean(axis=0)
        print(f"ID {tid}: center in *controller* frame = {ctr}, "
              f"x≈{ctr[0]:.3f} y≈{ctr[1]:.3f} z≈{ctr[2]:.3f}")
