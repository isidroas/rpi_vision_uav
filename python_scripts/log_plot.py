import pandas as pd
import plotly.express as px
df = pd.read_csv('build/example.csv')
#df.head()
fig = px.line(df, y = ['px','py'], title='X Pos')
fig.show()
