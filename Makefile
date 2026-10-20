# robot-follow-lab (spec v2.0 §61)
#
#   make build        イメージをビルド
#   make train-ai     Gazebo から学習データを生成して AI を学習し ONNX を出力
#   make demo         AI mode で追従デモを実行
#
# 学習済みモデルはリポジトリに含まれない (spec §5)。make demo は AI 学習を
# 暗黙に開始しない (spec §36)。

# コンテナをホストのユーザで動かす。これが無いと成果物が root 所有になり、
# 利用者が output/ を消せなくなる (DEVLOG S-21)
export HOST_UID := $(shell id -u)
export HOST_GID := $(shell id -g)

COMPOSE ?= docker compose
RUN_TRAIN = $(COMPOSE) --profile train run --rm trainer
RUN_TEST  = $(COMPOSE) --profile test  run --rm

.DEFAULT_GOAL := help
.PHONY: help build \
        generate-data train-model export-model validate-model train-ai \
        demo demo-marker demo-truth \
        smoke-test test validate-output \
        clean-data clean-model clean-output clean

help:
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	 | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'

# ソースはイメージに焼き込まれる (compose がマウントするのは
# models/ output/ data/ だけ)。編集を反映するには再ビルドが要る。
# 変更が無ければ 2 秒で終わるので、実行系の前に必ず通す (DEVLOG S-22)
.PHONY: image
image:
	@$(COMPOSE) build $(if $(VERBOSE),,--quiet)

build: ## イメージをビルドする
	$(COMPOSE) build

# ---------- AI 学習パイプライン (spec §33) ----------

generate-data: image ## Gazebo から学習データを生成する
	$(RUN_TRAIN) /workspace/scripts/generate_training_data.sh

train-model: image ## 生成済みデータで CNN を学習する
	$(RUN_TRAIN) /workspace/training/train_detector.py

export-model: image ## 学習済みモデルを ONNX へ変換する
	$(RUN_TRAIN) /workspace/training/export_onnx.py

validate-model: image ## ONNX / PyTorch の一致と推論性能を検証する
	$(RUN_TRAIN) /workspace/training/validate_model.py

check-ai-demo: image ## demo_world の実画像で AI の誤検出を検査する (DEVLOG X-3)
	$(RUN_TRAIN) /workspace/scripts/check_ai_on_demo.py

capture-demo-eval: image ## 誤検出検査用の demo_world 画像を収集し直す
	$(RUN_TEST) test /workspace/scripts/capture_demo_eval.sh

train-ai: image ## 上記 4 工程 + demo_world 誤検出検査を自動実行する (spec §6)
	$(RUN_TRAIN)

test-pipeline: image ## 小規模設定で train-ai が通ることを確認する (spec §64)
	TRAIN_IMAGES=100 TRAIN_EPOCHS=2 PIPELINE_TEST=1 $(RUN_TRAIN)

# ---------- デモ (spec §19) ----------

demo: image ## AI mode でデモを実行する
	PERCEPTION_MODE=ai $(COMPOSE) up --abort-on-container-exit demo

demo-marker: image ## marker mode (OpenCV 色抽出) でデモを実行する
	PERCEPTION_MODE=marker $(COMPOSE) up --abort-on-container-exit demo

demo-truth: image ## truth mode (Ground Truth, デバッグ専用) でデモを実行する
	PERCEPTION_MODE=truth $(COMPOSE) up --abort-on-container-exit demo

# ---------- テスト ----------

smoke-test: image ## 30-60 sim 秒の短時間動作確認 (spec §62)
	$(RUN_TEST) smoke

test: image ## unit test を実行する (spec §63)
	@# compose.yaml の検査は docker compose config を使うのでホスト側で走らせる。
	@# コンテナ内からは docker を呼べず、黙って skip されていた (DEVLOG S-23)
	@python3 scripts/check_compose_env.py
	$(RUN_TEST) test

video: image ## 既存の実験結果から combined.mp4 を作り直す (spec §57)
	$(RUN_TEST) test ros2 run video_compositor compose_video \
	  --run-dir $${RUN_DIR:-/workspace/output/latest}

validate-output: image ## output/ の成果物が揃っているか検証する
	$(RUN_TEST) test /workspace/scripts/validate_output.sh

# ---------- 後始末 ----------

clean-data: ## 生成した学習データを削除する
	rm -rf data/generated

clean-model: ## 学習済みモデルを削除する
	rm -f models/*.onnx models/*.pt models/training_report.json

clean-output: ## 実験結果を削除する
	rm -rf output/*/

clean: clean-data clean-model clean-output ## すべて削除する
