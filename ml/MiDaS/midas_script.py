import argparse
import os
import cv2
import numpy as np
import torch


def parse_args():
    parser = argparse.ArgumentParser(description="Monocular Depth Estimation using MiDaS")
    parser.add_argument("--image", type=str, required=True, help="Path to input image")
    parser.add_argument(
        "--model-type",
        type=str,
        default="MiDaS_small",
        choices=["MiDaS_small", "DPT_Hybrid", "DPT_Large"],
        help="MiDaS model architecture (default: MiDaS_small)",
    )
    parser.add_argument(
        "--weights",
        type=str,
        default=None,
        help="Optional path to local .pt / .pth checkpoint file",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="midas_depth_output.png",
        help="Path or directory for the output image",
    )
    parser.add_argument(
        "--colormap",
        type=str,
        default="inferno",
        choices=["inferno", "magma", "plasma", "viridis", "turbo"],
        help="Color grading palette (default: inferno)",
    )
    return parser.parse_args()


COLORMAP_DICT = {
    "inferno": cv2.COLORMAP_INFERNO,
    "magma": cv2.COLORMAP_MAGMA,
    "plasma": cv2.COLORMAP_PLASMA,
    "viridis": cv2.COLORMAP_VIRIDIS,
    "turbo": cv2.COLORMAP_TURBO,
}


def load_model(model_type, weights_path, device):
    pretrained = weights_path is None
    model = torch.hub.load("intel-isl/MiDaS", model_type, pretrained=pretrained)

    if weights_path is not None:
        state_dict = torch.load(weights_path, map_location=device)
        if "state_dict" in state_dict:
            state_dict = state_dict["state_dict"]
        # Strip prefixes from DataParallel training if present
        clean_state_dict = {
            k.replace("module.", "").replace("model.", ""): v
            for k, v in state_dict.items()
        }
        model.load_state_dict(clean_state_dict, strict=False)

    model.to(device)
    model.eval()
    return model


def main():
    args = parse_args()

    if not os.path.isfile(args.image):
        raise FileNotFoundError(f"Input image not found: {args.image}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 1. Load model and matching transform
    model = load_model(args.model_type, args.weights, device)
    midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

    if args.model_type in ["DPT_Large", "DPT_Hybrid"]:
        transform = midas_transforms.dpt_transform
    else:
        transform = midas_transforms.small_transform

    # 2. Read and preprocess image
    img_bgr = cv2.imread(args.image)
    orig_h, orig_w = img_bgr.shape[:2]
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    input_batch = transform(img_rgb).to(device)

    # 3. Depth inference
    with torch.no_grad():
        prediction = model(input_batch)
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=(orig_h, orig_w),
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    depth_map = prediction.cpu().numpy()

    # 4. Normalize to 8-bit dynamic range [0, 255]
    depth_norm = cv2.normalize(
        depth_map, None, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
    )

    # 5. Apply color grading
    cmap_code = COLORMAP_DICT.get(args.colormap, cv2.COLORMAP_INFERNO)
    depth_color = cv2.applyColorMap(depth_norm, cmap_code)

    # 6. Resolve output file path
    output_path = args.output
    if os.path.isdir(output_path) or not os.path.splitext(output_path)[1]:
        os.makedirs(output_path, exist_ok=True)
        base_name = os.path.splitext(os.path.basename(args.image))[0]
        output_path = os.path.join(output_path, f"{base_name}_depth.png")
    else:
        parent_dir = os.path.dirname(output_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

    cv2.imwrite(output_path, depth_color)
    print(f"Saved depth map to: {output_path}")


if __name__ == "__main__":
    main()
