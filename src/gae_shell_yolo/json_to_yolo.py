import os
import json
import shutil
import random
from glob import glob
from tqdm import tqdm

# --- AYARLAR ---
classes = ["road"]

# BURAYA KENDİ KLASÖR YOLLARINI YAZ
input_dirs = [
    "/home/otonom/furkan_wsyolo/saved_frames",
    "/home/otonom/furkan_wsyolo/saved_frames_engel_full",
]

output_dir = "yolo_dataset_road_engeldahil_yoladadahil"
val_split = 0.2

def convert_labelme_json(json_file):
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            content = f.read()
            if not content.strip():
                raise ValueError("Dosya boş")
            data = json.loads(content)
    except Exception as e:
        print(f"\nUYARI: Bozuk dosya atlandı -> {os.path.basename(json_file)} | Hata: {e}")
        return []
    
    img_w = data['imageWidth']
    img_h = data['imageHeight']
    yolo_lines = []

    for shape in data['shapes']:
        label = shape['label']
        if label not in classes:
            continue
        
        class_id = 0 
        points = shape['points']
        
        normalized_points = []
        for x, y in points:
            x = min(max(0, x), img_w)
            y = min(max(0, y), img_h)
            normalized_points.append(x / img_w)
            normalized_points.append(y / img_h)
            
        line_content = f"{class_id} " + " ".join([f"{p:.6f}" for p in normalized_points])
        yolo_lines.append(line_content)
        
    return yolo_lines

def create_folders():
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        
    for split in ['train', 'val']:
        os.makedirs(os.path.join(output_dir, split, 'images'), exist_ok=True)
        os.makedirs(os.path.join(output_dir, split, 'labels'), exist_ok=True)

def main():
    all_json_files = []
    
    print("Dosyalar taranıyor...")
    for folder in input_dirs:
        # Klasör yolunun doğruluğunu kontrol et
        if not os.path.exists(folder):
            print(f"UYARI: Klasör bulunamadı -> {folder}")
            continue
            
        files = glob(os.path.join(folder, "*.json"))
        print(f" -> '{folder}' içinde {len(files)} adet JSON bulundu.")
        all_json_files.extend(files)
    
    if not all_json_files:
        print(f"HATA: Belirtilen klasörlerde hiç .json dosyası bulunamadı!")
        return

    random.shuffle(all_json_files)
    create_folders()
    
    split_idx = int(len(all_json_files) * (1 - val_split))
    train_files = all_json_files[:split_idx]
    val_files = all_json_files[split_idx:]
    
    datasets = {'train': train_files, 'val': val_files}
    
    print(f"\nToplam {len(all_json_files)} dosya işlenecek.")
    
    for split, files in datasets.items():
        for json_file in tqdm(files, desc=f"{split} hazırlanıyor"):
            yolo_lines = convert_labelme_json(json_file)
            
            if not yolo_lines:
                continue

            base_name = os.path.splitext(os.path.basename(json_file))[0]
            input_dir_of_file = os.path.dirname(json_file)
            
            image_found = False
            src_img_path = ""
            # Olası uzantıları kontrol et
            for ext in [".png", ".jpg", ".jpeg", ".PNG", ".JPG"]:
                possible_path = os.path.join(input_dir_of_file, base_name + ext)
                if os.path.exists(possible_path):
                    src_img_path = possible_path
                    image_found = True
                    break
            
            # Hata veren kısım burasıydı, düzeltildi:
            if image_found:
                # Hedef dosya yolları
                dst_img_path = os.path.join(output_dir, split, 'images', os.path.basename(src_img_path))
                dst_txt_path = os.path.join(output_dir, split, 'labels', base_name + ".txt")
                
                # Kopyalama ve yazma işlemi
                shutil.copy(src_img_path, dst_img_path)
                with open(dst_txt_path, 'w') as f:
                    f.write("\n".join(yolo_lines))
            else:
                pass

    print(f"\nİşlem tamam! '{output_dir}' klasörü oluşturuldu.")
    abs_path = os.path.abspath(output_dir)
    print(f"path: {abs_path}")

if __name__ == "__main__":
    main()