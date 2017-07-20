source ~/Roboy/devel/setup.bash

roscd roboy_intents_classification

pip install -r requirements.txt

if [ ! -d data/models/ ]; then
	mkdir -p data/models
	cd data/models
	wget http://www.cs.toronto.edu/~rkiros/models/dictionary.txt
	wget http://www.cs.toronto.edu/~rkiros/models/utable.npy
	wget http://www.cs.toronto.edu/~rkiros/models/btable.npy
	wget http://www.cs.toronto.edu/~rkiros/models/uni_skip.npz
	wget http://www.cs.toronto.edu/~rkiros/models/uni_skip.npz.pkl
	wget http://www.cs.toronto.edu/~rkiros/models/bi_skip.npz
	wget http://www.cs.toronto.edu/~rkiros/models/bi_skip.npz.pkl
fi